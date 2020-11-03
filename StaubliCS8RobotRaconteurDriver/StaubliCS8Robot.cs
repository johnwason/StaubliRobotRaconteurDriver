using System;
using System.Collections.Generic;
using System.Text;
using RobotRaconteur;
using com.robotraconteur.robotics.robot;
using System.IO;
using System.Linq;
using com.robotraconteur.robotics.joints;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using com.robotraconteur.geometry;
using com.robotraconteur.action;
using com.robotraconteur.robotics.trajectory;
using RobotRaconteur.Companion.Robot;
using Staubli.Robotics.Soap.Proxies.ServerV0;
using Staubli.Robotics.Soap.Proxies.ServerV3;
using System.Globalization;
using Rox = GeneralRoboticsToolbox;
using System.Linq.Expressions;
using StaubliSoapClient;

namespace StaubliCS8RobotRaconteurDriver
{
    class StaubliCS8Robot : AbstractRobot
    {
        protected string _robot_host;
        protected ushort _robot_port;

        protected internal GeneralRoboticsToolbox.Robot[] rox_robots_no_limits;
        public StaubliCS8Robot(com.robotraconteur.robotics.robot.RobotInfo robot_info, string robot_host, ushort robot_port) : base(robot_info, -1)
        {
            _uses_homing = false;
            _has_position_command = false;
            _has_velocity_command = false;
            _has_jog_command = false;
            _update_period = 10;
            robot_info.robot_capabilities &= (uint)(RobotCapabilities.trajectory_command);

            _robot_host = robot_host;
            _robot_port = robot_port;

            this.rox_robots_no_limits = new Rox.Robot[robot_info.chains.Count];
            for (int i = 0; i < robot_info.chains.Count; i++)
            {
                this.rox_robots_no_limits[i] = RobotInfoConverter.ToToolboxRobot(robot_info, i);
                rox_robots_no_limits[i].Joint_lower_limit = null;
                rox_robots_no_limits[i].Joint_upper_limit = null;
                rox_robots_no_limits[i].Joint_vel_limit = null;
                rox_robots_no_limits[i].Joint_acc_limit = null;
            }

            m_address = "http://" + robot_host + ":" + robot_port;

        }

        CS8ServerV0PortTypeClient m_cs8ServerV0;
        CS8ServerV3PortTypeClient m_cs8ServerV3;
        int m_sessionId;
        string m_address;
        Staubli.Robotics.Soap.Proxies.ServerV0.Robot m_currentRobot;

        Thread _position_update_thread;

        public override void _start_robot()
        {

            m_cs8ServerV0 = new CS8ServerV0PortTypeClient();
            m_cs8ServerV0.Endpoint.Address = new System.ServiceModel.EndpointAddress(m_address);

            m_cs8ServerV3 = new CS8ServerV3PortTypeClient();
            m_cs8ServerV3.Endpoint.Address = new System.ServiceModel.EndpointAddress(m_address + "/CS8ServerV3");

            // If soap server is created, enable others tests
            if (m_cs8ServerV0 == null) throw new Exception("Could not connect to robot");
            // will generate an error if cannot connect to server
            SoapServerVersion l_soapVersion = m_cs8ServerV0.getSoapServerVersion("me", "0");
            double l_version = Convert.ToDouble(l_soapVersion.version, CultureInfo.GetCultureInfo("en-US").NumberFormat);

            m_cs8ServerV0.login("default", "", out m_sessionId);

            m_currentRobot = m_cs8ServerV0.getRobots(m_sessionId)[0];

            base._start_robot();

            _position_update_thread = new Thread(_position_feedback_thread_func);
            _position_update_thread.Start();
        }

        protected override Task _send_disable()
        {
            throw new NotImplementedException();
        }

        protected override Task _send_enable()
        {
            throw new NotImplementedException();
        }

        protected override Task _send_reset_errors()
        {
            throw new NotImplementedException();
        }

        protected override void _send_robot_command(long now, double[] joint_pos_cmd, double[] joint_vel_cmd)
        {

        }

        public override Task async_jog_cartesian(Dictionary<int, SpatialVelocity> velocity, double timeout, bool wait, int rr_timeout = -1)
        {
            throw new NotImplementedException();
        }

        public override Task async_jog_freespace(double[] joint_position, double[] max_velocity, bool wait, int timeout = -1)
        {
            throw new NotImplementedException();
        }

        public override Task async_jog_joint(double[] joint_velocity, double timeout, bool wait, int rr_timeout = -1)
        {
            throw new NotImplementedException();
        }

        protected void _position_feedback_thread_func()
        {
            while (_keep_going)
            {
                try
                {
                    double[] pos = m_cs8ServerV0.getRobotJointPos(m_sessionId, 0);

                    com.robotraconteur.geometry.Pose[] ep_pose = new com.robotraconteur.geometry.Pose[rox_robots_no_limits.Length];
                    try
                    {
                        for (int i = 0; i < rox_robots_no_limits.Length; i++)
                        {
                            var joint_numbers = this._robot_info.chains[i].joint_numbers;
                            double[] pos1 = new double[joint_numbers.Length];
                            for (int j = 0; j < joint_numbers.Length; j++)
                            {
                                pos1[j] = pos[joint_numbers[j]];
                            }
                            var rox_ep_pose = GeneralRoboticsToolbox.Functions.Fwdkin(rox_robots_no_limits[i], pos1);

                            ep_pose[i] = RobotRaconteur.Companion.Converters.GeometryConverter.ToPose(rox_ep_pose);
                        }
                    }
                    catch (Exception e)
                    {
                        ep_pose = null;
                    }

                    lock (this)
                    {
                        long now = _stopwatch.ElapsedMilliseconds;
                        _joint_position = pos;
                        _endpoint_pose = ep_pose;
                        _last_joint_state = now;
                        _last_endpoint_state = now;
                        _last_robot_state = now;
                        _ready = true;
                        _enabled = true;
                    }

                    Thread.Sleep(5);
                }
                catch (Exception e)
                {
                    Console.WriteLine("Error reading robot position data");
                    Console.WriteLine(e.Message);
                    Console.WriteLine(e.StackTrace);
                    Thread.Sleep(2000);
                }
            }
        }

        protected override bool _fill_robot_command(long now, out double[] joint_pos_cmd, out double[] joint_vel_cmd)
        {
            joint_pos_cmd = null;
            joint_vel_cmd = null;
            return false;
        }

        public override Task async_set_speed_ratio(double value, int timeout = -1)
        {
            throw new NotImplementedException();
        }

        StaubliCS8RobotTrajectoryTask _staubli_active_trajectory;

        public override Generator2<TrajectoryStatus> execute_trajectory(JointTrajectory trajectory)
        {
            lock (this)
            {
                if (_command_mode != RobotCommandMode.trajectory)
                {
                    throw new InvalidOperationException("Robot must be in trajectory mode to execute trajectory");
                }

                if (_staubli_active_trajectory != null && _staubli_active_trajectory.Running)
                {
                    throw new InvalidOperationException("Trajectory already running");
                }

                _staubli_active_trajectory = new StaubliCS8RobotTrajectoryTask();
                _staubli_active_trajectory.Init(trajectory, m_cs8ServerV3, m_sessionId);
                return _staubli_active_trajectory;
            }
        }
    }

    class StaubliCS8RobotTrajectoryTask : Generator2<com.robotraconteur.robotics.trajectory.TrajectoryStatus>
    {

        bool keep_going = false;

        CS8ServerV3PortTypeClient m_cs8ServerV3;
        int m_sessionId;
        com.robotraconteur.robotics.trajectory.JointTrajectory trajectory;

        bool aborted = false;
        bool stopped = false;
        TaskCompletionSource<int> done;

        public bool Running => keep_going;

        public void Init(com.robotraconteur.robotics.trajectory.JointTrajectory trajectory, CS8ServerV3PortTypeClient robot, int sessionid)
        {
            m_cs8ServerV3 = robot;
            m_sessionId = sessionid;
            this.trajectory = trajectory;
            done = new TaskCompletionSource<int>();
            keep_going = true;
        }

        public void Abort()
        {
            throw new NotImplementedException();
        }

        public Task AsyncAbort(int timeout = -1)
        {
            keep_going = false;
            aborted = true;
            return Task.FromResult(0);
        }

        public Task AsyncClose(int timeout = -1)
        {
            keep_going = false;
            stopped = true;
            return Task.FromResult(0);
        }

        Thread run_thread;

        public async Task<TrajectoryStatus> AsyncNext(int timeout = -1)
        {
            if (run_thread == null)
            {                
                run_thread = new Thread(run_thread_func);
                run_thread.Start();
                return new TrajectoryStatus()
                {
                    action_status = ActionStatusCode.running
                };
            }

            var wait_res = Task.WhenAny(done.Task, Task.Delay(10000));
            if (aborted)
            {
                throw new OperationAbortedException("");
            }
            if (stopped)
            {
                throw new StopIterationException("");
            }
            await wait_res;
            if (done.Task.IsCompleted || done.Task.IsFaulted || done.Task.IsCanceled)
            {
                await done.Task;
                throw new StopIterationException("");
                
            }

            return new TrajectoryStatus()
            {
                action_status = ActionStatusCode.running
            };
        }

        public void Close()
        {
            throw new NotImplementedException();
        }

        public TrajectoryStatus Next()
        {
            throw new NotImplementedException();
        }

        public TrajectoryStatus[] NextAll()
        {
            throw new NotImplementedException();
        }

        public void run_thread_func()
        {
            try
            {
                for (int i=0; i<trajectory.waypoints.Count; i++)
                {
                    if (!keep_going)
                    {
                        break;
                    }

                    var w = trajectory.waypoints[i];

                    MotionDesc l_mdesc = new MotionDesc();
                    l_mdesc.acc = 2.25;
                    l_mdesc.dec = 2.25;
                    l_mdesc.vel = 1.50;
                    l_mdesc.transVel = 99999;
                    l_mdesc.rotVel = 99999;
                    l_mdesc.freq = 250.0;
                    l_mdesc.absRel = MoveType.ABSOLUTEMOVE;
                    l_mdesc.config = new Config();
                    l_mdesc.config.Item = new AnthroConfig();
                    ((AnthroConfig)l_mdesc.config.Item).shoulder = ShoulderConfig.SSAME;
                    ((AnthroConfig)l_mdesc.config.Item).elbow = PositiveNegativeConfig.PNSAME;
                    ((AnthroConfig)l_mdesc.config.Item).wrist = PositiveNegativeConfig.PNSAME;
                    l_mdesc.frame = getAsIdentity();
                    l_mdesc.tool = getAsIdentity();
                    //l_mdesc.blendType = BlendType.BLENDOFF;
                    if (i < trajectory.waypoints.Count - 1)
                    {
                        l_mdesc.blendType = BlendType.BLENDJOINT;
                    }
                    else
                    {
                        l_mdesc.blendType = BlendType.BLENDOFF;
                    }
                    l_mdesc.distBlendPrev = (10 / 1000.0);       // in meter
                    l_mdesc.distBlendNext = (10 / 1000.0);       // in meter

                    MotionReturnCode l_rc;
                    //m_cs8ServerV3.resetMotion(m_sessionId);
                    m_cs8ServerV3.moveJJ(m_sessionId, 0, w.joint_position, l_mdesc, out l_rc);
                    if (l_rc != MotionReturnCode.SCSV3MOTNOERR)
                    {
                        throw new OperationFailedException($"Trajectory execution failed: {l_rc}");
                    }
                }
            }
            catch (Exception e)
            {
                keep_going = false;
                done.TrySetException(e);
                return;
            }
            keep_going = false;
            done.TrySetResult(0);
        }

        private static Frame getAsIdentity()
        {
            Frame l_ret = new Frame();
            Tools.setRxRyRzCoord(0, 0, 0, out l_ret);
            l_ret.px = 0;
            l_ret.py = 0;
            l_ret.pz = 0;
            return l_ret;
        }
    }
}
