#!/usr/bin/env python3
import rospy
import os
import json
import random
import re
import rospkg
from itertools import combinations
from std_msgs.msg import String, Float32
from jsk_2025_05_kashiwagi.srv import SetKashiwagiState
from ollama import chat

class ResponseGenerator:
    def __init__(self):
        rospy.init_node("propose_game_response_generator")
        rospy.sleep(1)
        self.json_path = os.path.join(os.path.dirname(__file__), "propose_game_word_card.json")
        self.path_to_pkg = rospkg.RosPack().get_path("jsk_2025_05_kashiwagi")
        self.tmp_response_path = os.path.join(self.path_to_pkg, "data", "tmp", "tmp_response.txt")
        self.last_qr_distance = None
        self.qr_distance_threshold = 0.10
        self.scanned_qr_ids = []
        self.required_qr_count = 6
        self.cur_state = "unknown"
        self.model_name = "gemma4:26b"
        self.round_finished = False
        self.card_cleared_since_finish = False
        self.child_safe_mode = True
        self.child_blocked_ids = {512, 632, 635, 638}

        self.initial_cards = {
            "group1":[{"id":"group1-a","text":"{君|きみ}にとって"},{"id":"group1-b","text":"{君|きみ}の"},{"id":"group1-c","text":"{君|きみ}と"},{"id":"group1-d","text":"{君|きみ}だけの"},{"id":"group1-e","text":"{君|きみ}を"}],
            "group2":[{"id":"group2-a","text":"僕にとって"},{"id":"group2-b","text":"僕の"},{"id":"group2-c","text":"僕と"},{"id":"group2-d","text":"僕だけの"},{"id":"group2-e","text":"僕を"}],
            "group3":[{"id":"group3-a","text":"僕に"},{"id":"group3-b","text":"僕"},{"id":"group3-c","text":"僕が"},{"id":"group3-d","text":"僕も"},{"id":"group3-e","text":"僕は"}],
            "group4":[{"id":"group4-a","text":"{君|きみ}に"},{"id":"group4-b","text":"{君|きみ}"},{"id":"group4-c","text":"{君|きみ}が"},{"id":"group4-d","text":"{君|きみ}も"},{"id":"group4-e","text":"{君|きみ}は"}],
            "group5":[{"id":"group5-a","text":"だけの"},{"id":"group5-b","text":"に"},{"id":"group5-c","text":"を"},{"id":"group5-d","text":"より"},{"id":"group5-e","text":"が"},{"id":"group5-f","text":"と"},{"id":"group5-g","text":"は"},{"id":"group5-h","text":"の"}],
            "group6":[{"id":"group6-a","text":"大切にするよ"},{"id":"group6-b","text":"愛してる"}]
        }

        self.word_map = {}
        self.speech_word_map = {}
        try:
            with open(self.json_path, encoding="utf-8") as f:
                data = json.load(f)
            for item in data:
                raw_text = item["text"]
                self.word_map[int(item["id"])] = self.to_display_text(raw_text)
                self.speech_word_map[int(item["id"])] = self.to_speech_text(raw_text)
            rospy.loginfo(f"succeeded in reading json: {len(self.word_map)} entries")
        except Exception as e:
            rospy.logerr(f"failed to read json: {e}")
            return

        self.initial_card_map = {}
        self.initial_speech_card_map = {}
        for cards in self.initial_cards.values():
            for card in cards:
                self.initial_card_map[card["id"]] = self.to_display_text(card["text"])
                self.initial_speech_card_map[card["id"]] = self.to_speech_text(card["text"])

        self.modifier_texts = {"消すことのできない","濡れた","ぽっかり空いた","命よりも大切な","本当の","終わらない","強がりな","おしとやかな","幸せな","誰も知らない","美しい","可愛い","あの日見た","じゃじゃ馬な","まぶしい"}
        self.adverb_texts = {"きっと","超","誰よりも","まるで","鬼のように","まったり","激しく","一瞬で","死ぬまで","これからもずっと","永久に","一緒に","スーパー","心から","マジ","世界一","絶対に","そろそろ"}
        self.state_texts = {"幸せ","ロマンティック","バラ色","素敵","ハッピー","尊い","最高","メロメロ","永劫不滅"}
        self.clause_texts = {"どんなに辛くても","神様がいるなら","苦しい時","悲しいとき","笑いながら","歩きながら","見つめられるだけで"}
        self.predicate_texts = {"結ばれる","どきどきする","一緒の墓に入ろう","見てみたい","抱きしめたい","叫んでる","昂ぶる","守るよ","離さない","幸せにしてみせる","暮らさないか？","受け取ってほしい","我慢できないんだ","耐えられないんだ","失いたくないんだ","一生一緒だよ","夢中さ","求めるのさ","したいんだ","聞いてくれ","作ってくれないか？","埋めてくれるかい？","いてほしい","どうにかなってしまいそうだ","幻想曲を奏でよう","足手纏いになるなよ","決して忘れはしない","深淵の中で揺れ動く","導いてくれ","言ったはずだ","運命の糸を交わそう","我が身を焦がす","契約を結ばん"}

        self.pub_response = rospy.Publisher("/propose_game_response", String, queue_size=10)
        self.pub_scan_progress = rospy.Publisher("/propose_game_scan_progress", String, queue_size=10)
        self.set_state_srv = rospy.ServiceProxy("/set_kashiwagi_state", SetKashiwagiState)
        rospy.Subscriber("/qr_distance", Float32, self.depth_update_callback, queue_size=1)
        rospy.Subscriber("/qr_data", String, self.response_callback, queue_size=1)
        rospy.Subscriber("/kashiwagi_state", String, self.state_callback, queue_size=1)
        rospy.loginfo("Propose Game Response Generator started")
        rospy.spin()

    def to_display_text(self, text):
        return re.sub(r"\{([^{}|]+)\|([^{}]+)\}", lambda m: m.group(1), text)

    def to_speech_text(self, text):
        return re.sub(r"\{([^{}|]+)\|([^{}]+)\}", lambda m: m.group(2), text)

    def state_callback(self, msg):
        old_state = self.cur_state
        self.cur_state = msg.data
        if old_state != self.cur_state:
            rospy.loginfo(f"kashiwagi state: {old_state} -> {self.cur_state}")

    def depth_update_callback(self, msg):
        self.last_qr_distance = msg.data
        if self.round_finished and msg.data > self.qr_distance_threshold:
            self.card_cleared_since_finish = True

    def change_state(self, state):
        try:
            resp = self.set_state_srv(state)
            if resp.success:
                rospy.loginfo(f"state updated: {resp.message}")
                return True
            rospy.logwarn(f"state update failed: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return False

    def reset_round(self):
        rospy.loginfo("===== RESET PROPOSE GAME =====")
        self.scanned_qr_ids = []
        self.round_finished = False
        self.card_cleared_since_finish = False
        self.publish_scan_progress()

    def response_callback(self, msg):
        if self.cur_state != "propose_game:playing":
            rospy.loginfo_throttle(5.0, f"QR ignored because current state is {self.cur_state}")
            return
        qr_text = msg.data.strip()
        rospy.loginfo(f"received QR data: {qr_text}")
        if self.last_qr_distance is None:
            rospy.logwarn("QR distance has not been received yet")
            return
        if self.last_qr_distance > self.qr_distance_threshold:
            rospy.logwarn(f"QR code is too far: {self.last_qr_distance:.3f} m")
            return
        if not qr_text.isdigit():
            rospy.logwarn("QR code data is not number")
            return

        qr_id = int(qr_text)
        if qr_id not in self.word_map:
            rospy.logwarn(f"Cannot find corresponding id: {qr_id}")
            return
        if self.child_safe_mode and qr_id in self.child_blocked_ids:
            rospy.logwarn(f"child-safe mode: blocked card {qr_id} -> {self.word_map[qr_id]}")
            return

        if self.round_finished:
            if self.card_cleared_since_finish or qr_id not in self.scanned_qr_ids:
                self.reset_round()
            else:
                rospy.loginfo("Waiting for current card to be removed before starting next round")
                return

        if len(self.scanned_qr_ids) >= self.required_qr_count:
            rospy.loginfo("Already collected 6 QR cards")
            return
        if qr_id in self.scanned_qr_ids:
            rospy.loginfo(f"QR {qr_id} has already been scanned")
            return

        self.scanned_qr_ids.append(qr_id)
        rospy.loginfo(f"QR: {qr_id} -> {self.word_map[qr_id]}")
        rospy.loginfo(f"QR count: {len(self.scanned_qr_ids)} / {self.required_qr_count}")
        self.publish_scan_progress()

        if len(self.scanned_qr_ids) == self.required_qr_count:
            rospy.loginfo("6 QR cards collected")
            self.change_state("propose_game:thinking_turn")
            self.generate_and_select_proposal()

    def publish_scan_progress(self):
        cards = [{"id":qr_id,"text":self.word_map[qr_id]} for qr_id in self.scanned_qr_ids]
        result = {"count":len(cards),"target":self.required_qr_count,"cards":cards}
        self.pub_scan_progress.publish(json.dumps(result, ensure_ascii=False))
        rospy.loginfo(f"scan progress published: {len(cards)} / {self.required_qr_count}")

    def validate_ids(self, ids):
        if not isinstance(ids, list):
            raise ValueError("ids is not a list")
        ids = [str(x) for x in ids]
        scanned_ids = [str(x) for x in self.scanned_qr_ids]
        used_qr_ids = [x for x in ids if x.isdigit()]
        if len(used_qr_ids) not in [3, 4, 5, 6]:
            raise ValueError(f"must use 3 to 6 QR cards: {len(used_qr_ids)}")
        if len(used_qr_ids) != len(set(used_qr_ids)):
            raise ValueError("same QR card used multiple times")
        for qr_id in used_qr_ids:
            if qr_id not in scanned_ids:
                raise ValueError(f"QR id was not scanned: {qr_id}")
            if self.child_safe_mode and int(qr_id) in self.child_blocked_ids:
                raise ValueError(f"blocked QR card used: {qr_id}")
        allowed_ids = set(scanned_ids) | set(self.initial_card_map.keys())
        for card_id in ids:
            if card_id not in allowed_ids:
                raise ValueError(f"invalid card id: {card_id}")
        used_groups = set()
        for card_id in ids:
            if card_id.startswith("group"):
                group = card_id.split("-")[0]
                if group in used_groups:
                    raise ValueError(f"multiple cards used from {group}")
                used_groups.add(group)
        if len(ids) > 10:
            raise ValueError("too many cards")
        return ids

    def get_card_text(self, card_id):
        card_id = str(card_id)
        return self.initial_card_map[card_id] if card_id.startswith("group") else self.word_map[int(card_id)]

    def get_speech_card_text(self, card_id):
        card_id = str(card_id)
        return self.initial_speech_card_map[card_id] if card_id.startswith("group") else self.speech_word_map[int(card_id)]

    def ids_to_proposal(self, ids):
        return " ".join(f'"{self.get_card_text(card_id)}"' for card_id in ids)

    def is_sentence_ending_card(self, card_id):
        card_id = str(card_id)
        if card_id.startswith("group6"):
            return True
        if card_id.startswith("group"):
            return False
        text = self.get_card_text(card_id)
        endings = ("だよ","だね","だろ？","かな？","ないか？","くれ","ほしい","したいんだ","してみせる","するよ","愛してる","なのさ","なんだね","ですか？","変かな？","かもしれない","離さない","守るよ","一生一緒だよ","我慢できないんだ","耐えられないんだ","失いたくないんだ","夢中さ","そう思うだろ？","どうにかなってしまいそうだ","になってくれないか？","になりたいのさ","作ってくれないか？","受け取ってほしい","いてほしい","暮らさないか？","埋めてくれるかい？","聞いてくれ","ダメですか？","あったかいだろ？","求めるのさ","あげるよ","なり","止められん","砕け散るがいい","者よ","くれてやる","疼いてやがる","舞い踊る")
        return text in self.predicate_texts or text.endswith(endings)

    def card_type(self, card_id):
        card_id = str(card_id)
        text = self.get_card_text(card_id)
        if card_id.startswith("group5"): return "particle"
        if text in ["僕は","僕が","僕も","君は","君が","君も"]: return "subject"
        if text in ["僕","君","我","汝"]: return "pronoun"
        if text in ["僕の","僕だけの","君の","君だけの"]: return "possessive"
        if text in ["僕を","僕に","君を","君に","僕と","君と","僕にとって","君にとって"]: return "case_phrase"
        if not card_id.startswith("group") and (text.startswith("の") or text.startswith("にな") or text.startswith("に変") or text.startswith("にそっと") or text.startswith("って") or text in ["でしかない","しかない","みたいだね","なんだね","なり","は神すら止められん","で砕け散るがいい","に囚われし者よ","の半分をくれてやる","が疼いてやがる","と共に舞い踊る"]): return "suffix_predicate"
        if self.is_sentence_ending_card(card_id): return "predicate"
        if text in self.modifier_texts: return "modifier"
        if text in self.adverb_texts: return "adverb"
        if text in self.state_texts: return "state"
        if text in self.clause_texts: return "clause"

        noun_ids = {505,509,513,517,520,523,524,529,532,533,536,538,539,543,545,549,551,553,556,557,562,566,567,571,578,580,582,586,592,594,605,609,612,615,623,626,632,636,639,643,644,646,650,653,655,661,671,672,673,674,675}
        modifier_ids = {506,512,528,537,568,577,583,591,602,620,621,631,652,657}
        adverb_ids = {501,504,508,511,541,555,561,584,585,590,599,619,628,634,637,645,647,659}
        if not card_id.startswith("group"):
            qr_id = int(card_id)
            if qr_id in modifier_ids: return "modifier"
            if qr_id in adverb_ids: return "adverb"
            if qr_id in noun_ids: return "noun"
        return "other"

    def terminal_like(self, card_id):
        return self.card_type(card_id) in ["predicate","suffix_predicate"] or str(card_id).startswith("group6")

    def preferred_position(self, card_id):
        positions = {"subject":0.10,"pronoun":0.16,"possessive":0.22,"clause":0.22,"case_phrase":0.34,"modifier":0.38,"noun":0.50,"state":0.68,"particle":0.60,"adverb":0.62,"suffix_predicate":0.82,"predicate":0.90}
        return positions.get(self.card_type(card_id), 0.50)

    def pair_score(self, a, b):
        a_text, b_text = self.get_card_text(a), self.get_card_text(b)
        a_type, b_type = self.card_type(a), self.card_type(b)
        score = 0.0
        if a_type == "possessive" and b_type == "noun": score += 16.0
        if a_type == "modifier" and b_type == "noun": score += 15.0
        if a_type == "noun" and b_type == "particle": score += 8.0
        if a_type in ["noun","state","pronoun"] and b_type == "suffix_predicate": score += 12.0
        if a_type == "case_phrase" and b_type in ["predicate","state"]: score += 10.0
        if a_type == "adverb" and b_type in ["predicate","state","modifier"]: score += 7.0
        if a_type == "subject" and b_type in ["possessive","case_phrase","modifier","noun","state","adverb"]: score += 6.0
        if a_type == "clause" and b_type in ["subject","case_phrase","adverb","predicate","state"]: score += 5.0
        if a_type == "particle" and b_type == "particle": score -= 70.0
        if a_type in ["predicate","suffix_predicate"] and b_type in ["predicate","suffix_predicate"]: score -= 70.0
        if a_type == "possessive" and b_type != "noun": score -= 80.0
        if a_type == "modifier" and b_type != "noun": score -= 80.0
        if a_type == "subject" and b_type in ["subject","pronoun"]: score -= 50.0
        if a_type == "pronoun" and b_type in ["subject","pronoun","possessive"]: score -= 45.0
        if a_type in ["predicate","suffix_predicate"] and b_type in ["subject","pronoun","possessive","particle","modifier"]: score -= 80.0
        if a_text == "の" and b_type != "noun": score -= 80.0
        if a_text == "だけの" and b_type != "noun": score -= 80.0
        if a_text == "を" and b_type in ["predicate","suffix_predicate"]: score += 9.0
        if a_text == "を" and b_type in ["noun","particle","possessive","subject","pronoun"]: score -= 50.0
        if a_text in ["が","は"] and b_type in ["predicate","state","modifier","noun","adverb"]: score += 7.0
        if a_text in ["が","は"] and b_type in ["particle","possessive"]: score -= 50.0
        if a_text == "に" and b_type in ["predicate","state"]: score += 7.0
        if a_text == "の" and b_type == "noun": score += 10.0
        if a_text in ["僕は","僕が","僕も"] and b_text in ["僕","僕の","僕だけの"]: score -= 50.0
        if a_text in ["君は","君が","君も"] and b_text in ["君","君の","君だけの"]: score -= 50.0
        if a_text in ["僕は","僕が","僕も"] and b_text in ["君の","君だけの","君を","君に","君にとって"]: score += 8.0
        if a_text in ["君を","君に","僕を","僕に"] and b_type in ["predicate","state"]: score += 8.0
        if a_text in ["君と","僕と"] and b_text in ["一緒に","手を取り合って","これからもずっと"]: score += 8.0
        if a_text == "同じ苗字" and b_text in ["になりたいのさ","になってくれないか？"]: score += 18.0
        if a_text == "同じ苗字" and b_text == "を": score -= 35.0
        if a_text == "美しい" and b_type == "noun": score += 8.0
        if a_text == "メロメロ" and b_type == "noun": score -= 20.0
        if a_text in ["最高","素敵","メロメロ","バラ色"] and b_type in ["noun","modifier"]: score -= 15.0
        return score

    def triple_score(self, a, b, c):
        a_text, b_text, c_text = self.get_card_text(a), self.get_card_text(b), self.get_card_text(c)
        a_type, b_type, c_type = self.card_type(a), self.card_type(b), self.card_type(c)
        score = 0.0
        if a_type in ["possessive","modifier"] and b_type == "noun" and c_type == "particle": score += 15.0
        if a_type == "subject" and b_type == "possessive" and c_type == "noun": score += 12.0
        if a_type == "subject" and b_type == "case_phrase" and c_type in ["predicate","state"]: score += 14.0
        if a_type == "noun" and b_type == "particle" and c_type in ["predicate","state","suffix_predicate"]: score += 14.0
        if a_type == "modifier" and b_type == "noun" and c_type in ["particle","predicate","suffix_predicate"]: score += 11.0
        if a_type == "case_phrase" and b_type == "adverb" and c_type in ["predicate","state"]: score += 8.0
        if a_type in ["predicate","suffix_predicate"] and b_type in ["predicate","suffix_predicate"]: score -= 70.0
        if a_type == "subject" and b_type in ["subject","pronoun"]: score -= 50.0
        if a_type in ["predicate","suffix_predicate"] and b_type in ["subject","pronoun","possessive"]: score -= 70.0
        if a_text == "同じ苗字" and b_text == "を" and c_text == "大切にするよ": score -= 70.0
        if a_text == "美しい" and b_text == "同じ苗字" and c_text == "を": score -= 45.0
        return score

    def extension_score(self, prefix, next_id, total_len):
        new_seq = prefix + [next_id]
        next_type = self.card_type(next_id)
        pos = (len(new_seq) - 1) / max(total_len - 1, 1)
        score = -abs(pos - self.preferred_position(next_id)) * 2.0
        if len(new_seq) == 1:
            if next_type == "particle": score -= 80.0
            if next_type == "suffix_predicate": score -= 80.0
            if next_type == "predicate": score -= 40.0
        if len(new_seq) >= 2: score += self.pair_score(new_seq[-2], new_seq[-1])
        if len(new_seq) >= 3: score += self.triple_score(new_seq[-3], new_seq[-2], new_seq[-1])
        remaining = total_len - len(new_seq)
        if self.terminal_like(next_id) and remaining > 0: score -= 22.0 + min(remaining, 3) * 6.0
        if str(next_id).startswith("group6") and remaining > 0: score -= 45.0
        content_types = {"noun","modifier","pronoun","other","state"}
        chain = 0
        for card_id in reversed(new_seq):
            if self.card_type(card_id) in content_types: chain += 1
            else: break
        if chain >= 3: score -= 14.0 * (chain - 2)
        return score

    def candidate_score(self, ids):
        ids = [str(x) for x in ids]
        n = len(ids)
        score = 0.0
        pair_total = sum(self.pair_score(ids[i], ids[i + 1]) for i in range(n - 1))
        triple_total = sum(self.triple_score(ids[i], ids[i + 1], ids[i + 2]) for i in range(n - 2))
        score += pair_total / max(n - 1, 1)
        score += 0.8 * triple_total / max(n - 2, 1)
        position_error = sum(abs((i / max(n - 1, 1)) - self.preferred_position(card_id)) for i, card_id in enumerate(ids))
        score -= 3.0 * position_error / max(n, 1)
        if self.terminal_like(ids[-1]): score += 12.0
        else: score -= 10.0
        for card_id in ids[:-1]:
            if self.is_sentence_ending_card(card_id): score -= 24.0
            if str(card_id).startswith("group6"): score -= 40.0
        if str(ids[-1]).startswith("group6"): score += 8.0
        content_types = {"noun","modifier","pronoun","other","state"}
        chain = 0
        for card_id in ids:
            if self.card_type(card_id) in content_types:
                chain += 1
                if chain >= 3: score -= 10.0 * (chain - 2)
            else:
                chain = 0
        initial_count = sum(str(card_id).startswith("group") for card_id in ids)
        if initial_count == 2: score += 0.5
        elif initial_count == 3: score += 2.5
        elif initial_count == 4: score += 1.5
        qr_count = sum(str(card_id).isdigit() for card_id in ids)
        score += 0.5 * (qr_count - 3)
        return score

    def beam_order_cards(self, card_ids, beam_width=80, result_count=5):
        total_len = len(card_ids)
        beam = [([], tuple(card_ids), 0.0)]
        for _ in range(total_len):
            expanded = []
            for prefix, remaining, score in beam:
                for i, next_id in enumerate(remaining):
                    new_prefix = prefix + [next_id]
                    new_remaining = remaining[:i] + remaining[i + 1:]
                    new_score = score + self.extension_score(prefix, next_id, total_len)
                    expanded.append((new_prefix, new_remaining, new_score))
            expanded.sort(key=lambda x: x[2], reverse=True)
            beam = expanded[:beam_width]
        completed = [item[0] for item in beam]
        completed.sort(key=self.candidate_score, reverse=True)
        return completed[:result_count]

    def choose_initial_ids(self):
        initial_count = random.choice([2,3,3,3,4,4])
        group_names = list(self.initial_cards.keys())
        mandatory = []
        if random.random() < 0.90: mandatory.append("group6")
        if initial_count >= 3 and random.random() < 0.60: mandatory.append("group5")
        mandatory = mandatory[:initial_count]
        remaining_groups = [group for group in group_names if group not in mandatory]
        selected_groups = mandatory + random.sample(remaining_groups, initial_count - len(mandatory))
        return [random.choice(self.initial_cards[group])["id"] for group in selected_groups]

    def generate_legal_candidates(self, num_candidates=120):
        scanned_ids = [str(x) for x in self.scanned_qr_ids]
        qr_sets = {count:[list(x) for x in combinations(scanned_ids, count)] for count in [3,4,5]}
        qr_sets[6] = [scanned_ids.copy()]
        card_sets, seen_sets = [], set()
        attempts = 0
        while len(card_sets) < 180 and attempts < 5000:
            attempts += 1
            used_qr_count = random.choice([3,4,5,6])
            used_qr = random.choice(qr_sets[used_qr_count]).copy()
            initial_ids = self.choose_initial_ids()
            cards = used_qr + initial_ids
            key = tuple(sorted(cards))
            if key in seen_sets: continue
            try: self.validate_ids(cards)
            except Exception: continue
            seen_sets.add(key)
            card_sets.append(cards)

        candidates, seen_candidates = [], set()
        for index, cards in enumerate(card_sets):
            ordered_candidates = self.beam_order_cards(cards, beam_width=80, result_count=5)
            for ids in ordered_candidates:
                key = tuple(ids)
                if key in seen_candidates: continue
                try: ids = self.validate_ids(ids)
                except Exception: continue
                seen_candidates.add(key)
                candidates.append(ids)
            if (index + 1) % 20 == 0:
                rospy.loginfo(f"beam search: {index + 1} / {len(card_sets)} card sets")

        candidates.sort(key=self.candidate_score, reverse=True)
        candidates_2 = [ids for ids in candidates if sum(str(x).startswith("group") for x in ids) == 2]
        candidates_3 = [ids for ids in candidates if sum(str(x).startswith("group") for x in ids) == 3]
        candidates_4 = [ids for ids in candidates if sum(str(x).startswith("group") for x in ids) == 4]
        selected = candidates_2[:25] + candidates_3[:60] + candidates_4[:35]
        selected.sort(key=self.candidate_score, reverse=True)
        candidates = selected[:num_candidates]

        rospy.loginfo(f"Beam Search generated {len(seen_candidates)} candidates, selected top {len(candidates)}")
        rospy.loginfo("===== TOP 15 BEAM CANDIDATES =====")
        for i, ids in enumerate(candidates[:15]):
            used_qr = {int(x) for x in ids if x.isdigit()}
            unused = [x for x in self.scanned_qr_ids if x not in used_qr]
            initial_count = sum(str(x).startswith("group") for x in ids)
            rospy.loginfo(f"{i}: score={self.candidate_score(ids):.2f} qr={len(used_qr)} initial={initial_count} unused={unused} {self.ids_to_proposal(ids)}")
        return candidates

    def select_from_batch(self, candidates):
        candidate_data = [{"number":i,"proposal":self.ids_to_proposal(ids),"qr_count":sum(x.isdigit() for x in ids),"initial_count":sum(x.startswith("group") for x in ids)} for i, ids in enumerate(candidates)]

        prompt = f"""以下の候補から、日本語として最も自然で意味が通り、ロマンチックなプロポーズを必ず1つ選んでください。

候補:
{json.dumps(candidate_data, ensure_ascii=False)}

このゲームは小学生を含む参加者と遊びます。
「キッス」「抱きしめたい」「好き」「愛してる」程度の健全な恋愛表現は問題ありません。
露骨な性的表現、強い下ネタ、強い暴言は避けてください。
まず日本語として自然に読めるかを評価してください。
完全に完璧な文章でなくても構いません。
カードゲームなので多少ユニークな表現や詩的な表現は許容してください。
多少変な候補しかない場合でも、その中で最も自然で意味が通るものを必ず選んでください。

評価順序:
1. 日本語として自然
2. 文全体として意味を理解できる
3. カード同士の接続が自然
4. プロポーズとして愛情が伝わる
5. ロマンチック、面白い、印象に残る
6. 小学生を含む場でも安心して読める

QRカードは3〜6枚のいずれでもよく、枚数は評価しないでください。
基本カードの枚数も評価しないでください。
候補を変更、並べ替え、追加、削除してはいけません。
必ず0以上のnumberを1つだけ選んでください。
JSONだけで回答してください。
{{"selected":0}}"""

        for attempt in range(3):
            try:
                response = chat(model=self.model_name, messages=[
                    {"role":"system","content":"候補の中から最も自然で意味が通り、プロポーズとして魅力的なものを必ず1つ選んでください。多少ユニークな文章でも許容してください。候補そのものは変更せず番号だけを選んでください。"},
                    {"role":"user","content":prompt}
                ], format="json", think=False, stream=False, options={"temperature":0.0,"num_predict":40})
                content = response["message"]["content"]
                rospy.loginfo(f"selection response: {content}")
                result = json.loads(content)
                selected = int(result["selected"])
                if selected < 0 or selected >= len(candidates):
                    raise ValueError(f"invalid selected candidate: {selected}")
                return candidates[selected]
            except Exception as e:
                rospy.logwarn(f"selection failed ({attempt + 1}/3): {e}")
        rospy.logwarn("Gemma selection failed, using best heuristic candidate")
        return candidates[0]

    def select_best_candidate(self, candidates):
        batch_size = 12
        winners = []
        for start in range(0, len(candidates), batch_size):
            batch = candidates[start:start + batch_size]
            batch_number = start // batch_size + 1
            rospy.loginfo(f"Gemma selecting batch {batch_number}: {len(batch)} candidates")
            winner = self.select_from_batch(batch)
            winners.append(winner)
            rospy.loginfo(f"batch {batch_number} winner: {self.ids_to_proposal(winner)}")
        if len(winners) == 1:
            return winners[0]
        rospy.loginfo(f"Selecting final winner from {len(winners)} finalists")
        return self.select_from_batch(winners)

    def build_chunks(self, ids):
        ids = [str(x) for x in ids]
        chunks, current = [], []
        for i, card_id in enumerate(ids):
            current.append(card_id)
            next_id = ids[i + 1] if i + 1 < len(ids) else None
            current_type = self.card_type(card_id)
            next_type = self.card_type(next_id) if next_id is not None else None
            current_text = self.get_card_text(card_id)
            should_break = False
            if self.is_sentence_ending_card(card_id): should_break = True
            elif next_type == "subject" and len(current) >= 2: should_break = True
            elif current_text.endswith(("ながら","想うと","触れると")): should_break = True
            elif len(current) >= 5 and current_type in ["noun","particle","other","state"]: should_break = True
            elif next_id is None: should_break = True
            if should_break:
                chunks.append({"ids":[int(x) if x.isdigit() else x for x in current],"texts":[self.get_card_text(x) for x in current],"text":"".join(self.get_card_text(x) for x in current)})
                current = []
        return chunks

    def build_speech_text(self, chunks):
        parts = []
        for i, chunk in enumerate(chunks):
            ids = chunk.get("ids", [])
            text = "".join(self.get_speech_card_text(card_id) for card_id in ids).strip()
            if not text: continue
            last_id = str(ids[-1]) if ids else None
            is_last_chunk = i == len(chunks) - 1
            if text.endswith(("。","！","？","!","?")): parts.append(text)
            elif text.endswith("、"): parts.append(text)
            elif last_id is not None and self.is_sentence_ending_card(last_id): parts.append(text + "。")
            elif is_last_chunk: parts.append(text + "。")
            else: parts.append(text + "、")
        return "".join(parts) + "結婚しよう。"

    def write_speech_text(self, speech_text):
        try:
            os.makedirs(os.path.dirname(self.tmp_response_path), exist_ok=True)
            with open(self.tmp_response_path, "w", encoding="utf-8") as f:
                f.write(speech_text)
            rospy.loginfo(f"speech text written: {self.tmp_response_path}")
            return True
        except Exception as e:
            rospy.logwarn(f"failed to write speech text: {e}")
            return False

    def generate_and_select_proposal(self):
        try:
            candidates = self.generate_legal_candidates(120)
            if not candidates:
                raise ValueError("No legal candidates generated")

            rospy.loginfo("Selecting best proposal with Gemma 4")
            best_ids = self.validate_ids(self.select_best_candidate(candidates))
            proposal = self.ids_to_proposal(best_ids)
            chunks = self.build_chunks(best_ids)
            speech_text = self.build_speech_text(chunks)
            used_qr_ids = {int(x) for x in best_ids if x.isdigit()}
            unused_qr_ids = [x for x in self.scanned_qr_ids if x not in used_qr_ids]
            initial_count = sum(str(x).startswith("group") for x in best_ids)

            final_result = {
                "proposal":proposal,
                "speech":speech_text,
                "ids":[int(x) if x.isdigit() else x for x in best_ids],
                "used_qr_count":len(used_qr_ids),
                "used_initial_count":initial_count,
                "unused_ids":unused_qr_ids,
                "chunks":chunks,
                "heuristic_score":self.candidate_score(best_ids)
            }

            final_json = json.dumps(final_result, ensure_ascii=False)
            rospy.loginfo("===== SELECTED PROPOSAL =====")
            rospy.loginfo(final_json)
            for i, chunk in enumerate(chunks):
                rospy.loginfo(f"chunk {i}: {chunk['text']}")
            rospy.loginfo(f"speech: {speech_text}")
            print(json.dumps(final_result, ensure_ascii=False, indent=2))

            # HTMLはこの時点ですぐ完成形を表示
            self.pub_response.publish(final_json)

            # 発話文だけファイルに準備する。まだVoiceVoxは動かさない
            if not self.write_speech_text(speech_text):
                raise RuntimeError("failed to prepare speech text")

            # 「柏木さんの番だよ」を待つ
            self.change_state("propose_game:ready_to_speak")
            self.round_finished = True
            self.card_cleared_since_finish = False
            rospy.loginfo("Proposal is ready. Waiting for '柏木さんの番だよ'.")

        except Exception as e:
            rospy.logerr(f"failed to generate/select proposal: {e}")

if __name__ == "__main__":
    try:
        ResponseGenerator()
    except rospy.ROSInterruptException:
        pass
