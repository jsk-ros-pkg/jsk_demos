const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
const Scratch3RobotBase = require('../scratch3_ros/RobotUtil');
const {RosUtil} = require('../scratch3_ros/RosUtil');
const ROSLIB = require('roslib');
const Cast = require('../../util/cast');
const icon = require('./icon');

class Scratch3PepperRobotBlocks extends Scratch3RobotBase {

    constructor(runtime, extensionId) {
        super('Pepper', extensionId ? extensionId : 'pepperRobot',
              '/robotsound', '/robotsound_jp', runtime);
        this.icon = icon;

        this.joint_server = '/pepper_robot/pose/joint_trajectory';
        this.joint_names = ['KneePitch', 'HipRoll', 'HipPitch', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'HeadYaw', 'HeadPitch'];

        this.poses = {
            'nodding-pose': [0.034906585039886584, -0.034906585039886584, -0.08726646259971643, 1.919862177193762, 0.6108652381980151, -0.6981317007977315, -1.0471975511965974, -0.6981317007977315, 1.919862177193762, -0.6108652381980151, 0.6981317007977315, 1.0471975511965974, 0.6981317007977315, 0.0, 0.26179938779914935],

            'pointing-left-pose': [0.034906585039886584, 0.0, -0.08726646259971643, 0.8726646259971642, 1.396263401595463, -1.5707963267948957, -0.34906585039886573, -1.5707963267948957, 1.4835298641951793, -0.17453292519943286, 1.2217304763960302, 0.34906585039886573, 0.6981317007977315, 0.5235987755982987, -0.1047197551196597],

            'pointing-right-pose': [0.034906585039886584, 0.0, -0.08726646259971643, 1.4835298641951793, 0.17453292519943286, -1.2217304763960302, -0.34906585039886573, -0.6981317007977315, 0.8726646259971642, -1.396263401595463, 1.5707963267948957, 0.34906585039886573, 1.5707963267948957, -0.5235987755982987, -0.06981317007977317],

            'reset-pose': [0.034906585039886584, -0.034906585039886584, -0.08726646259971643, 1.4835298641951793, 0.17453292519943286, -1.2217304763960302, -0.34906585039886573, -0.6981317007977315, 1.4835298641951793, -0.17453292519943286, 1.2217304763960302, 0.34906585039886573, 0.6981317007977315, 0.0, 0.0],

            'watching-pose': [0.034906585039886584, -0.034906585039886584, -0.08726646259971643, 1.7453292519943284, 0.8726646259971642, -0.34906585039886573, -1.396263401595463, -0.6981317007977315, -1.0471975511965974, -0.34906585039886573, -0.17453292519943286, 1.0471975511965974, 0.6981317007977315, 0.0, 0.0],

            'wondering-pose': [0.034906585039886584, -0.34906585039886573, -0.08726646259971643, 1.7453292519943284, 0.8726646259971642, -0.34906585039886573, -1.396263401595463, -0.6981317007977315, -1.0471975511965974, -0.34906585039886573, -0.17453292519943286, 1.0471975511965974, 0.6981317007977315, -0.08726646259971643, 0.0],

            'c-pose': [0.034906585039886584, -0.034906585039886584, -0.08726646259971643, 0.0, 1.2217304763960302, -1.2217304763960302, -0.34906585039886573, -0.6981317007977315, -0.6981317007977315, -0.17453292519943286, 0.34906585039886573, 1.2217304763960302, 0.6981317007977315, 0.0, -0.34906585039886573],

            'hiragana-tsu-pose': [0.034906585039886584, -0.034906585039886584, -0.08726646259971643, -0.6981317007977315, 0.17453292519943286, -0.34906585039886573, -1.2217304763960302, -0.6981317007977315, 0.0, -1.2217304763960302, 1.2217304763960302, 0.34906585039886573, 0.6981317007977315, 0.0, -0.34906585039886573],

            'bowing-pose': [0.034906585039886584, -0.034906585039886584, -0.6981317007977315, 1.4835298641951793, 0.17453292519943286, -1.2217304763960302, -0.34906585039886573, -0.6981317007977315, 0.8726646259971642, -0.017453292519943292, -0.017453292519943292, 0.8726646259971642, 0.8726646259971642, 0.0, 0.34906585039886573],

            'ok-pose': [0.034906585039886584, -0.034906585039886584, -0.08726646259971643, -0.6981317007977315, 0.17453292519943286, -0.34906585039886573, -1.2217304763960302, -0.6981317007977315, -0.6981317007977315, -0.17453292519943286, 0.34906585039886573, 1.2217304763960302, 0.6981317007977315, 0.0, -0.34906585039886573],

            'greeting-pose': [0.034906585039886584, -0.034906585039886584, -0.08726646259971643, 1.4835298641951793, 0.17453292519943286, 0.5235987755982987, -0.008726646259971646, 0.5235987755982987, -0.34906585039886573, -0.34906585039886573, 1.2217304763960302, 0.34906585039886573, 0.6981317007977315, 0.0, -0.26179938779914935],

            'init-pose': [0.034906585039886584, -0.034906585039886584, -0.08726646259971643, 1.919862177193762, 0.6108652381980151, -0.6981317007977315, -1.0471975511965974, -0.6981317007977315, 1.919862177193762, -0.6108652381980151, 0.6981317007977315, 1.0471975511965974, 0.6981317007977315, 0.0, 0.26179938779914935]
        }
    }

    connect (url) {
        this.ros = new RosUtil(this.runtime, this.extensionId, {url: url});
        this.ros.on('connection', () => {
            console.log("Setting joint server...");
            this._setJointServer();
        });
    }

    _toRange(val, min, max) {
        return parseInt( min + (max - min)*((val+180)/360.0) );
    }

    _makeModifier(key, value) {
        return "\\\\" + key + "=" + value + "\\\\";
    }

    _makeJointRequest(positions, time) {
        if (time < 1) time = 1;
        let secs = Math.floor(time);
        let nsecs = parseInt((time-secs) * 1000000000);
        return {
            trajectory: {
                joint_names: this.joint_names,
                points: [
                    {
                        positions: positions,
                        velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        time_from_start: {
                            secs: secs,
                            nsecs: nsecs,
                        }
                    } ]
            }
        };
    }

    _setJointServer () {
        if (typeof this.joint_server == 'string') {
            this.joint_server = new ROSLIB.ActionClient({
                ros: this.ros,
                serverName: this.joint_server,
                actionName: 'naoqi_bridge_msgs/JointTrajectoryAction'
            });
        }
    }

    _poseNames () {
        return Object.keys(this.poses).sort();
    }

    speech({TEXT}) {
        const msg = {data: TEXT};
        return this.ros.publishTopic('/speech', msg).
            catch(err => this._reportError(err));
    }

    pitchModifier({VALUE}) {
        let val = Cast.toNumber(VALUE);
        val = this._toRange(val, 50, 200);
        return this._makeModifier('vct', val);
    }

    rateModifier({VALUE}) {
        let val = Cast.toNumber(VALUE);
        val = this._toRange(val, 50, 400);
        return this._makeModifier('rspd', val);
    }

    volumeModifier({VALUE}) {
        let val = Cast.toNumber(VALUE);
        val = this._toRange(val, 0, 100);
        return this._makeModifier('vol', val);
    }

    pauseModifier({VALUE}) {
        let val = Cast.toNumber(VALUE);
        return this._makeModifier('pau', val);
    }

    resetModifier() {
        return ('\\\\rst\\\\');
    }

    takePose({POSE, TIME, WAIT}) {
        TIME = Cast.toNumber(TIME);
        WAIT = Cast.toBoolean(WAIT);
        let pose = this.poses[POSE];
        let msg = this._makeJointRequest(pose, TIME);
        this._setJointServer();
        return this._waitPromise(this.ros.callActionInstance(this.joint_server, msg, WAIT)).
            catch(err => this._reportError(err));
    }

    hello () {
	return this.ros.callService('/hello', {});
    }

    greeting () {
	return this.ros.callService('/greeting', {});
    }

    look_at_kochisan () {
	return this.ros.callService('/look_at_kochisan', {});
    }

    listening () {
	return this.ros.callService('/listening', {});
    }

    episode_introduction () {
	return this.ros.callService('/episode_introduction', {});
    }

    episode_0_1 () {
	return this.ros.callService('/episode_0_1', {});
    }

    episode_0_2 () {
	return this.ros.callService('/episode_0_2', {});
    }

    episode_1_1 () {
	return this.ros.callService('/episode_1_1', {});
    }

    episode_1_2 () {
	return this.ros.callService('/episode_1_2', {});
    }

    episode_1_3 () {
	return this.ros.callService('/episode_1_3', {});
    }

    episode_1_4 () {
	return this.ros.callService('/episode_1_4', {});
    }

    episode_2 () {
	return this.ros.callService('/episode_2', {});
    }

    episode_3_1 () {
	return this.ros.callService('/episode_3_1', {});
    }

    episode_3_2 () {
	return this.ros.callService('/episode_3_2', {});
    }

    episode_3_3 () {
	return this.ros.callService('/episode_3_3', {});
    }

    episode_4_1 () {
	return this.ros.callService('/episode_4_1', {});
    }

    episode_4_2 () {
	return this.ros.callService('/episode_4_2', {});
    }

    episode_5_1 () {
	return this.ros.callService('/episode_5_1', {});
    }

    episode_5_2 () {
	return this.ros.callService('/episode_5_2', {});
    }

    episode_summary () {
	return this.ros.callService('/episode_summary', {});
    }

    episode_6 () {
	return this.ros.callService('/episode_6', {});
    }

    end_greeting () {
	return this.ros.callService('/end_greeting', {});
    }

    getInfo () {
        return {
            id: this.extensionId,
            name: this.extensionName,
            showStatusButton: true,

            color1: this._randomizedColor(0xBED3D7),
            menuIconURI: this.icon,
            
            blocks: [
                {
                    opcode: 'query',
                    blockType: BlockType.BOOLEAN,
                    text: 'query [TEXT]',
                    arguments: {
                        TEXT: {
                            type: ArgumentType.STRING,
                            defaultValue: 'Proceed?'
                        }
                    }
                },
                {
                    opcode: 'speech',
                    blockType: BlockType.COMMAND,
                    text: 'speak [TEXT]',
                    arguments: {
                        TEXT: {
                            type: ArgumentType.STRING,
                            defaultValue: 'Hello!'
                        }
                    }
                },
                {
                    opcode: 'pitchModifier',
                    blockType: BlockType.REPORTER,
                    text: 'set pitch to [VALUE]',
                    arguments: {
                        VALUE: {
                            type: ArgumentType.ANGLE,
                            defaultValue: -60
                        }
                    }
                },
                {
                    opcode: 'rateModifier',
                    blockType: BlockType.REPORTER,
                    text: 'set rate to [VALUE]',
                    arguments: {
                        VALUE: {
                            type: ArgumentType.ANGLE,
                            defaultValue: -130
                        }
                    }
                },
                {
                    opcode: 'volumeModifier',
                    blockType: BlockType.REPORTER,
                    text: 'set volume to [VALUE]',
                    arguments: {
                        VALUE: {
                            type: ArgumentType.ANGLE,
                            defaultValue: 100
                        }
                    }
                },
                {
                    opcode: 'pauseModifier',
                    blockType: BlockType.REPORTER,
                    text: 'pause speech by [VALUE] ms',
                    arguments: {
                        VALUE: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 50
                        }
                    }
                },
                '---',
                {
                    opcode: 'takePose',
                    blockType: BlockType.COMMAND,
                    text: 'pose [POSE] in [TIME] s wait: [WAIT]',
                    arguments: {
                        POSE: {
                            type: ArgumentType.STRING,
                            menu: 'poseMenu',
                            defaultValue: this._poseNames()[0]
                        },
                        TIME: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 5
                        },
                        WAIT: {
                            type: ArgumentType.BOOLEAN,
                            menu: 'booleanMenu',
                            defaultValue: 'true'
                        }
                    }
                },
		'---',
		{
		    opcode: 'hello',
		    blockType: BlockType.COMMAND,
		    text: 'hello',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'greeting',
		    blockType: BlockType.COMMAND,
		    text: 'greeting',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'look_at_kochisan',
		    blockType: BlockType.COMMAND,
		    text: 'look_at_kochisan',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'listening',
		    blockType: BlockType.COMMAND,
		    text: 'listening',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_introduction',
		    blockType: BlockType.COMMAND,
		    text: 'episode_introduction',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_0_1',
		    blockType: BlockType.COMMAND,
		    text: 'episode_0_1',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_0_2',
		    blockType: BlockType.COMMAND,
		    text: 'episode_0_2',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_1_1',
		    blockType: BlockType.COMMAND,
		    text: 'episode_1_1',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_1_2',
		    blockType: BlockType.COMMAND,
		    text: 'episode_1_2',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_1_3',
		    blockType: BlockType.COMMAND,
		    text: 'episode_1_3',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_1_4',
		    blockType: BlockType.COMMAND,
		    text: 'episode_1_4',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_2',
		    blockType: BlockType.COMMAND,
		    text: 'episode_2',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_3_1',
		    blockType: BlockType.COMMAND,
		    text: 'episode_3_1',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_3_2',
		    blockType: BlockType.COMMAND,
		    text: 'episode_3_2',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_3_3',
		    blockType: BlockType.COMMAND,
		    text: 'episode_3_3',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_4_1',
		    blockType: BlockType.COMMAND,
		    text: 'episode_4_1',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_4_2',
		    blockType: BlockType.COMMAND,
		    text: 'episode_4_2',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_5_1',
		    blockType: BlockType.COMMAND,
		    text: 'episode_5_1',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_5_2',
		    blockType: BlockType.COMMAND,
		    text: 'episode_5_2',
		    arguments: {
		        }
		},
		'---',
		{
		    opcode: 'episode_summary',
		    blockType: BlockType.COMMAND,
		    text: 'episode_summary',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'episode_6',
		    blockType: BlockType.COMMAND,
		    text: 'episode_6',
		    arguments: {
			}
		},
		'---',
		{
		    opcode: 'end_greeting',
		    blockType: BlockType.COMMAND,
		    text: 'end_greeting',
		    arguments: {
			}
		},
            ],
            menus: {
                booleanMenu: ['true', 'false'],
                poseMenu: '_poseNames'
            }
        };
    }
}

module.exports = Scratch3PepperRobotBlocks;
