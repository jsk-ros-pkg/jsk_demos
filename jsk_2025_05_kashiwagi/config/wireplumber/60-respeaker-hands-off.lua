-- Keep PipeWire away from the ReSpeaker.
--
-- respeaker_node opens the array directly with PyAudio on hw:, and the echo
-- canceller takes its reference from inside sound_play's playback pipeline, so
-- nothing in this robot's audio path goes through PipeWire. Leaving the card
-- claimed here only means two owners for one USB interface: on 2026-08-20 the
-- array logged 42 USB resets and twelve failed descriptor reads in a single
-- session, in bursts that landed on node start times and carried
-- "usb_set_interface failed (-32)", while the speaker and the keyboard on the
-- same hub logged none. Repeatedly re-opening a UAC1.0 interface is what the
-- XVF-3000 firmware does not survive, and it eventually stopped enumerating
-- at all.
--
-- Disabling the device here does not affect arecord or PyAudio: they talk to
-- ALSA, and this only stops the session manager from claiming the card.

table.insert(alsa_monitor.rules, {
  matches = {
    {
      { "device.name", "matches", "alsa_card.usb-SEEED_ReSpeaker*" },
    },
  },
  apply_properties = {
    ["device.disabled"] = true,
  },
})
