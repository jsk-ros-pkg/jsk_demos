#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random, time, sys, signal, copy, math
import numpy as np
import argparse

import chainer
from chainer import Variable, Link, Chain, ChainList, optimizers, serializers
import chainer.functions as F
import chainer.links as L
from chainer.functions.loss.mean_squared_error import mean_squared_error

from matplotlib import pyplot as plt
import seaborn as sns

import rospy
from std_msgs.msg import Float64MultiArray, Float64

LOG_FILES = ['../log/log-by-logger/log-by-loggerpy1_0.log',
             '../log/log-by-logger/log-by-loggerpy1_1.log',
             '../log/log-by-logger/log-by-loggerpy1_2.log',
             '../log/log-by-logger/log-by-loggerpy1_4.log',
             '../log/log-by-logger/log-by-loggerpy1_5.log',
             '../log/log-by-logger/log-by-loggerpy1_7.log',    
             '../log/log-by-logger/log-by-loggerpy1_6.log',
             '../log/log-by-logger/log-by-loggerpy1_3.log']

class MyChain(Chain):
    def __init__(self):
        super(MyChain, self).__init__(   # FIX
            l1=L.Linear(6, 2),
            l2=L.Linear(2)
            )
        
    def forward(self, x):   # FIX
        #h = F.sigmoid(self.l1(x))
        h = self.l1(x)
        o = self.l2(h)
        return o

    # forward and save output
    def __call__(self, x):
        self.res = self.forward(x)

    # loss func for learning
    def loss(self, t):
        return F.mean_squared_error(self.res, t)
    
    # loss for optimize input
    def loss_for_optimize_input(self, t):
        return F.mean_squared_error(self.res, t)

class DiaboloSystem():
    def __init__(self):
        # PAST NUM
        self.PAST_STATE_NUM = 2   # FIX
        self.PAST_INPUT_NUM = 1   # FIX
        self.PAST_NUM = max(self.PAST_STATE_NUM, self.PAST_INPUT_NUM)

        # INPUT DIM
        self.STATE_DIM = 2
        self.INPUT_DIM = 2
        
        self.DELTA_STEP = 5   # FIX

        # HyperParameters for NeuralNetwork
        self.INPUT_NN_DIM = self.STATE_DIM * self.PAST_STATE_NUM + self.INPUT_DIM * self.PAST_INPUT_NUM
        self.OUTPUT_NN_DIM = self.STATE_DIM
        self.TRAIN_TEST_RATIO = 0.8   # FIX
        self.BATCH_SIZE = 1000   # FIX

        # reference of state
        self.state_ref = [0., 0.]   # FIX

        self.MPC_PREDICT_STEP = 10
        self.online_training = False
        self.LPF_AVERAGE_NUM = 10
        self.idle_flag = 0

        # real data
        self.past_states = []
        self.past_inputs = []
        self.now_input = [0.7, 0]

        # max min restriction for input
        self.MAX_INPUT_DIFF_RESTRICTION = [0.03, 0.01]
        self.MAX_INPUT_RESTRICTION = [0.85, 0.34]
        self.MIN_INPUT_RESTRICTION = [0.60, -0.34]

        # init ros variable
        rospy.init_node("DiaboloSystem")
        self.pub_diabolo_input = rospy.Publisher('/diabolo_system/diabolo_input', Float64MultiArray, queue_size=1)
        self.pub_diabolo_state = rospy.Publisher('/diabolo_system/diabolo_state', Float64MultiArray, queue_size=1)        

    # print percentage of progress every 10%
    def percentage(self, idx, loop_num):
        split_num = 10
        if (idx + 1) % int(loop_num / split_num) == 0:
            print('{}% {}/{}'.format((idx + 1) * 100 / loop_num, idx + 1, loop_num))

    # load data from log files
    def load_data(self, log_files):
        self.input_arm = []
        self.input_base = []
        self.state_pitch = []
        self.state_yaw = []

        self.input_arm_lpf = []
        self.input_base_lpf = []
        self.state_pitch_lpf = []
        self.state_yaw_lpf = []
        
        for log_file in log_files:
            with open(log_file, "r") as f:
                cnt = 0
                for l in f.readlines():
                    cnt += 1
                    val = l.split(' ')
                    if len(val) != 4:
                        break
                
                    self.input_arm.append(float(val[0]))
                    self.input_base.append(float(val[1]))
                    self.state_pitch.append(float(val[2]))
                    self.state_yaw.append(float(val[3][:-1]))

                    if cnt > self.LPF_AVERAGE_NUM:
                        a = 0
                        b = 0
                        p = 0
                        y = 0
                        for i in range(self.LPF_AVERAGE_NUM):
                            a += self.input_arm[-i]
                            b += self.input_base[-i]
                            p += self.state_pitch[-i]
                            y += self.state_yaw[-i]
                        self.input_arm_lpf.append(a / self.LPF_AVERAGE_NUM)
                        self.input_base_lpf.append(b / self.LPF_AVERAGE_NUM)
                        self.state_pitch_lpf.append(p / self.LPF_AVERAGE_NUM)
                        self.state_yaw_lpf.append(y / self.LPF_AVERAGE_NUM)

    # arrange data for NeuralNetwork
    def arrange_data(self):
        self.X = []
        self.Y = []

        use_lpf = True
        if use_lpf: # for LPF
            for i in range((max(self.PAST_STATE_NUM, self.PAST_INPUT_NUM) + 1) * self.DELTA_STEP, len(self.input_arm_lpf)):
                x = []
                y = [self.state_pitch_lpf[i], self.state_yaw_lpf[i]]            
                for j in range(self.PAST_STATE_NUM):
                    x.append(self.state_pitch_lpf[i - (j + 1) * self.DELTA_STEP])
                    x.append(self.state_yaw_lpf[i - (j + 1) * self.DELTA_STEP])
                for j in range(self.PAST_INPUT_NUM):
                    x.append(self.input_arm_lpf[i - (j + 1) * self.DELTA_STEP])
                    x.append(self.input_base_lpf[i - (j + 1) * self.DELTA_STEP])
                self.X.append(x)
                self.Y.append(y)
        else: # for not LPF
            for i in range((max(self.PAST_STATE_NUM, self.PAST_INPUT_NUM) + 1) * self.DELTA_STEP, len(self.input_arm)):
                x = []
                y = [self.state_pitch[i], self.state_yaw[i]]            
                for j in range(self.PAST_STATE_NUM):
                    x.append(self.state_pitch[i - (j + 1) * self.DELTA_STEP])
                    x.append(self.state_yaw[i - (j + 1) * self.DELTA_STEP])
                for j in range(self.PAST_INPUT_NUM):
                    x.append(self.input_arm[i - (j + 1) * self.DELTA_STEP])
                    x.append(self.input_base[i - (j + 1) * self.DELTA_STEP])
                self.X.append(x)
                self.Y.append(y)

    # make NeuralNetwork model by using MyChain class
    def make_model(self):
        self.model = MyChain()
        self.optimizer = optimizers.RMSprop(lr=0.01)
        self.optimizer.setup(self.model)

    def get_batch_train(self, n):
        x = []
        y = []
        for i in range(n):
            r = (int(random.random() * 10 * len(self.X) * self.TRAIN_TEST_RATIO) % int(len(self.X) * self.TRAIN_TEST_RATIO))
            x.append(self.X[r])
            y.append(self.Y[r])
        return np.array(x), np.array(y)
    
    def get_test(self, log_files = None):
        x = []
        y = []
        if log_files == None:
            s = int(len(self.X) * self.TRAIN_TEST_RATIO)
            for i in range(len(self.X) - s):
                x.append(self.X[s + i])
                y.append(self.Y[s + i])
        else:
            self.load_data(log_files)
            self.arrange_data()
            for i in range(len(self.X)):
                x.append(self.X[i])
                y.append(self.Y[i])
        return np.array(x), np.array(y)

    # load trained NeuralNetwork model
    def load_model(self, log_file='../log/diabolo_system/mymodel.h5'):
        serializers.load_hdf5(log_file, self.model)
        
        # plot weight heatmap
        self.draw_heatmap(self.model)

    # save trained NeuralNetwork model
    def save_model(self):
        serializers.save_hdf5('../log/diabolo_system/mymodel.h5', self.model)
        
    def train(self, loop_num=1000):
        # train loop_num
        losses =[]        
        for i in range(loop_num):
            self.percentage(i, loop_num)
            x, y = self.get_batch_train(self.BATCH_SIZE)
        
            x_ = Variable(x.astype(np.float32).reshape(self.BATCH_SIZE, 6))
            t_ = Variable(y.astype(np.float32).reshape(self.BATCH_SIZE, 2))

            self.model.zerograds()
            self.model(x_)
            loss = self.model.loss(t_)

            loss.backward()
            self.optimizer.update()
            losses.append(loss.data)

        # plot loss
        print('[Train] loss is {}'.format(losses[-1]))
        plt.title('Loss')
        plt.xlabel('iteration')
        plt.ylabel('loss')                
        plt.plot(losses)
        plt.yscale('log')
        plt.show()

        # plot weight heatmap
        self.draw_heatmap(self.model)
            
    def test(self):
        # x, y = self.get_test(['../log/log-by-logger/log-by-loggerpy_short-string_1.log'])
        # x, y = self.get_test(['../log/log-by-logger/log-by-loggerpy_diabolo-system_2.log'])
        # x, y = self.get_test(['../log/log-by-logger/log-by-loggerpy1_6.log'])
        x, y = self.get_test()
        x_ = Variable(x.astype(np.float32).reshape(len(x),6))
        t_ = Variable(y.astype(np.float32).reshape(len(y),2))
        
        self.model.zerograds()
        self.model(x_)
        loss = self.model.loss(t_)
        # loss.backward() # this should be comment out
        # self.optimizer.update()
        
        with open('../log/diabolo_system/predict.log', 'w') as f:
            for i in range(len(x_)):
                f.write('{} {} {} {} {} {}\n'.format(x_[i][4].data, x_[i][5].data, t_[i][0].data, t_[i][1].data, self.model.res[i][0].data, self.model.res[i][1].data))
        print('[Test] loss is {}'.format(loss.data))

    def realtime_feedback(self, simulate=False, online_training=False):
        self.online_training = online_training
        self.online_plot = online_training and False
        print('[RealtimeFeedback] online training is {}'.format(self.online_training))        
        
        print('[RealtimeFeedback] reference of diabolo state is {}'.format(self.state_ref))
        if simulate == True:
            #init_state = [0., 0.]
            init_state = [40., 0.]

            self.past_states = [init_state for i in range(self.PAST_STATE_NUM * self.DELTA_STEP)]            
            while True:
                self.simulate_once()
                self.publish_input()
                self.publish_state()
                time.sleep(1. / 20) # wait 20Hz                
        else:
            # for realtime plot
            if self.online_training == True:
                if self.online_plot == True:
                    plt.ion()
                    plt.figure()
                
                    plt.title("Loss")
                    self.li, = plt.plot(np.zeros(100), np.zeros(100))
                    plt.xlabel("time[step]")
                    plt.ylabel("loss")
                
                self.online_losses = []

                # make optimizer for online training
                self.optimizer_online = optimizers.RMSprop(lr=0.001)
                self.optimizer_online.setup(self.model)
            
            self.past_states = []
            self.past_inputs = []
            
            self.input_arm_lpf = []
            self.input_base_lpf = []
            self.state_pitch_lpf = []
            self.state_yaw_lpf = []
            
            rospy.Subscriber("idle", Float64, self.callback_for_idle, queue_size=1)
            rospy.Subscriber("calc_idle_diabolo_state/diabolo_state", Float64MultiArray, self.callback_for_state, queue_size=1)
            r = rospy.Rate(30)
            
            #for realtime plot of online training loss
            cnt = 0
            while True:
                cnt += 1
                if cnt % 100 == 0:
                    self.save_model()
                r.sleep()
                if self.online_training == True:
                    if self.online_plot == True:
                        plot_num = min(100, len(self.online_losses) - 1)
                        losses = copy.deepcopy(self.online_losses[-plot_num:]) # prevent from not same length between x and y because of async callback
                        self.li.set_xdata(np.array([i for i in range(len(losses))]))
                        self.li.set_ydata(losses)
                        plt.xlim(0, len(losses))
                        if len(self.online_losses) > 0:
                            plt.ylim(min(self.online_losses), max(self.online_losses[-int(plot_num / 2.):]) * 2)
                        plt.draw()
                        plt.pause(0.001)
            # rospy.spin()
            # callback_for_state is working background
            #   optimize_input
            #   publish_input
            #   subscribe state
            #   arrange subscribed data for train

    def callback_for_idle(self, msg):
        self.idle_flag = msg.data
        
    def callback_for_state(self, msg):
        # assign past_states
        self.past_states.append([msg.data[0], msg.data[1]])

        # assign lpf state and input
        if len(self.past_states) > self.LPF_AVERAGE_NUM and len(self.past_inputs) > self.LPF_AVERAGE_NUM:
            a = 0
            b = 0
            p = 0
            y = 0
            for i in range(self.LPF_AVERAGE_NUM):
                a += self.past_inputs[-i][0]
                b += self.past_inputs[-i][1]
                p += self.past_states[-i][0]
                y += self.past_states[-i][1]
            self.input_arm_lpf.append(a / self.LPF_AVERAGE_NUM)
            self.input_base_lpf.append(b / self.LPF_AVERAGE_NUM)
            self.state_pitch_lpf.append(p / self.LPF_AVERAGE_NUM)
            self.state_yaw_lpf.append(y / self.LPF_AVERAGE_NUM)
        
        # optimize input
        optimize_input_start_time = time.time()                
        if len(self.past_states) > self.PAST_STATE_NUM * self.DELTA_STEP:
            self.optimize_input()
            print('{} {} {} {}'.format(self.now_input[0], self.now_input[1], self.past_states[-1][0], self.past_states[-1][1]))
            self.publish_input()
        optimize_input_end_time = time.time()            

        # online training
        train_start_time = time.time()        
        batch_num = 10
        if self.online_training == True:
            if self.idle_flag < 1:
                return
            print('Now Online Training')
            # arrange data for train
            if len(self.input_arm_lpf) < (max(self.PAST_STATE_NUM, self.PAST_INPUT_NUM) + 1) * self.DELTA_STEP + batch_num:
                return

            X = []
            Y = []
            for i in range(batch_num):
                x = []
                y = [self.state_pitch_lpf[-i], self.state_yaw_lpf[-i]]            
                for j in range(self.PAST_STATE_NUM):
                    x.append(self.state_pitch_lpf[-i - (j + 1) * self.DELTA_STEP])
                    x.append(self.state_yaw_lpf[-i - (j + 1) * self.DELTA_STEP])
                for j in range(self.PAST_INPUT_NUM):
                    x.append(self.input_arm_lpf[-i - (j + 1) * self.DELTA_STEP])
                    x.append(self.input_base_lpf[-i - (j + 1) * self.DELTA_STEP])
                X.append(x)
                Y.append(y)
            
            # train
            loop_num = 1
            losses = 0
            for i in range(loop_num):
                x_ = Variable(np.array(X).astype(np.float32).reshape(batch_num, 6))
                t_ = Variable(np.array(Y).astype(np.float32).reshape(batch_num, 2))
            
                self.model.zerograds()
                self.model(x_)
                loss = self.model.loss(t_)
    
                loss.backward()
                self.optimizer_online.update()

                losses += loss.data
    
            self.online_losses.append(losses / loop_num / batch_num)
                
        train_end_time = time.time()
        print('optimize input : {}[s], online training : {}[s]   Is this lower than 0.033?'.format(optimize_input_end_time - optimize_input_start_time, train_end_time - train_start_time))
        if 0.33 < train_end_time - train_start_time + optimize_input_end_time - optimize_input_start_time:
            print('[Error] time out of optimize input and online training')
    
    def publish_input(self):
        msg = Float64MultiArray()
        msg.data = [self.now_input[0], self.now_input[1]]
        self.pub_diabolo_input.publish(msg)

    def publish_state(self):
        msg = Float64MultiArray()
        msg.data = [self.past_states[-1][0], self.past_states[-1][1]]
        self.pub_diabolo_state.publish(msg)

    # CHECK
    def optimize_input(self):
        x = Variable(np.array([self.past_states[-1 * self.DELTA_STEP], self.past_states[-2 * self.DELTA_STEP], self.now_input]).astype(np.float32).reshape(1,6))
        t = Variable(np.array(self.state_ref).astype(np.float32).reshape(1,2))
        loop_flag = True
        for i in range(20):    # optimize loop  loop_num is 10 == hz is 90
            self.model.zerograds()            
            self.model(x)
            # loss = self.model.loss(t)
            loss = self.model.loss_for_optimize_input(t)
            loss.backward()
            
            x = Variable((x - 0.01 * x.grad_var).data)
            now_input = [x[0][4].data, x[0][5].data]
            # apply input restriction
            for j in range(self.PAST_INPUT_NUM * self.INPUT_DIM):
                # diff input restriction
                if now_input[j] - self.now_input[j] > self.MAX_INPUT_DIFF_RESTRICTION[j]:
                    now_input[j] = np.float32(self.now_input[j] + self.MAX_INPUT_DIFF_RESTRICTION[j])
                    #loop_flag = False
                elif self.now_input[j] - now_input[j] > self.MAX_INPUT_DIFF_RESTRICTION[j]:
                    now_input[j] = np.float32(self.now_input[j] - self.MAX_INPUT_DIFF_RESTRICTION[j])
                    #loop_flag = False                    
                # max min input restriction
                if now_input[j] > self.MAX_INPUT_RESTRICTION[j]:
                    now_input[j] = self.MAX_INPUT_RESTRICTION[j]
                    #loop_flag = False                    
                elif now_input[j] < self.MIN_INPUT_RESTRICTION[j]:
                    now_input[j] = self.MIN_INPUT_RESTRICTION[j]
                    #loop_flag = False                    
              
            x = Variable(np.array([self.past_states[-1 * self.DELTA_STEP], self.past_states[-2 * self.DELTA_STEP], now_input]).astype(np.float32).reshape(1,6))
            if loop_flag == False:
                break

        self.now_input = [float(x[0][self.PAST_STATE_NUM * self.STATE_DIM + 0].data), float(x[0][self.PAST_STATE_NUM * self.STATE_DIM + 1].data)]
        self.past_inputs.append([self.now_input[0], self.now_input[1]])

    def simulate_once(self): # optimize input and simulate
        self.optimize_input()
        now_x = Variable(np.array([self.past_states[-1 * self.DELTA_STEP], self.past_states[-2 * self.DELTA_STEP], self.now_input]).astype(np.float32).reshape(1, 6))
        
        self.model(now_x)
        res = self.model.res
        now_state = [res[0][0].data, res[0][1].data]
        self.past_states.append(now_state)

    def simulate_offline(self, simulate_loop_num=1000):
        #init_state = [0., 0.]
        init_state = [40., 0.]        
        
        self.past_states = [init_state for i in range(self.PAST_STATE_NUM * self.DELTA_STEP)]
        with open('../log/diabolo_system/simulate.log', 'w') as f:
            for i in range(simulate_loop_num):   # simulation loop
                self.percentage(i, simulate_loop_num)                
                self.simulate_once()
                f.write('{} {} {} {}\n'.format(self.now_input[0], self.now_input[1], self.past_states[-1][0], self.past_states[-1][1]))
                          
    def draw_heatmap(self, model):
        wb1 = np.c_[model.l1.W.data, model.l1.b.data]
        wb2 = np.c_[model.l2.W.data, model.l2.b.data]
        wbs = [wb1, wb2]
        for i in range(len(wbs)):
            wb = wbs[i]
            plt.subplot(len(wbs) ,1 ,i + 1)            
            sns.heatmap(wb, annot=True, cmap='Blues')
        plt.show()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, lambda signal, frame: sys.exit(0))

    # init arg parser
    parser = argparse.ArgumentParser()
    parser.add_argument("--train", "-t", nargs='?', default=False, const=True, help="train NN")
    parser.add_argument("--action", "-a", default=2, help="0:simulate 1:realtime feedback with simulate 2:realtime feedback with real robot")
    parser.add_argument("--model", "-m", default='../log/diabolo_system/mymodel.h5', help="which model do you use")
    parser.add_argument("--online_training", "-o", nargs='?', default=False, const=True, help="online training")                
    args = parser.parse_args()

    # parse
    train_flag = int(args.train)   # parse train
    action = int(args.action)   # parse action
    model_file = args.model   # which model
    online_training_ = args.online_training   # which model
    
    ds = DiaboloSystem()
    
    # train model or load model
    if train_flag:
        ds.load_data(LOG_FILES)
        ds.arrange_data()
        ds.make_model()
        print('[Train] start')        
        ds.train(loop_num=500)
        ds.save_model()
    else:
        ds.load_data(LOG_FILES)
        ds.arrange_data()
        ds.make_model()
        print('[Train] pass')                
        ds.load_model(log_file=model_file)
        print('load model from {}'.format(model_file))
        
    # test
    print('[Test] start')            
    ds.test()

    # action
    if action == 0:
        print('[Simulate] start')
        ds.simulate_offline(simulate_loop_num=300)
    elif action == 1:
        print('[RealtimeFeedback] start with simulate')                
        ds.realtime_feedback(simulate=True, online_training=False)
    elif action == 2:
        print('[RealtimeFeedback] start with real robot')                
        ds.realtime_feedback(simulate=False, online_training=online_training_)
