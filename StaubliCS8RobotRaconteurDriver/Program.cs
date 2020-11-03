using Mono.Options;
using RobotRaconteur.Companion.InfoParser;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RobotRaconteur;
using Mono.Unix;

namespace StaubliCS8RobotRaconteurDriver
{
    class Program
    {
        static int Main(string[] args)                
        {

            bool shouldShowHelp = false;
            string robot_info_file = null;
            bool wait_signal = false;
            string robot_hostname = null;
            ushort? robot_port = null;

            var options = new OptionSet {
                { "robot-info-file=", n => robot_info_file = n },
                { "robot-hostname=", n => robot_hostname = n },
                { "robot-port=", n => robot_port = ushort.Parse(n) },
                { "h|help", "show this message and exit", h => shouldShowHelp = h != null },
                {"wait-signal", "wait for POSIX sigint or sigkill to exit", n=> wait_signal = n!=null}
            };

            List<string> extra;
            try
            {
                // parse the command line
                extra = options.Parse(args);
            }
            catch (OptionException e)
            {
                // output some error message
                Console.Write("StaubliCS8RobotRaconteurDriver: ");
                Console.WriteLine(e.Message);
                Console.WriteLine("Try `StaubliCS8RobotRaconteurDriver --help' for more information.");
                return 1;
            }

            if (shouldShowHelp)
            {
                Console.WriteLine("Usage: StaubliCS8RobotRaconteurDriver [Options+]");
                Console.WriteLine();
                Console.WriteLine("Options:");
                options.WriteOptionDescriptions(Console.Out);
                return 0;
            }

            if (robot_info_file == null)
            {
                Console.WriteLine("error: robot-info-file must be specified");
                return 1;
            }


            var robot_info = RobotInfoParser.LoadRobotInfoYamlWithIdentifierLocks(robot_info_file);
            using (robot_info.Item2)
            {



                using (var robot = new StaubliCS8Robot(robot_info.Item1,robot_hostname,robot_port.Value))
                {
                    robot._start_robot();
                    using (var node_setup = new ServerNodeSetup("ABB_robot", 58649, args))
                    {


                        RobotRaconteurNode.s.RegisterService("robot", "com.robotraconteur.robotics.robot", robot);

                        if (!wait_signal)
                        {
                            Console.WriteLine("Press enter to exit");
                            Console.ReadKey();
                        }
                        else
                        {
                            UnixSignal[] signals = new UnixSignal[]{
                                new UnixSignal (Mono.Unix.Native.Signum.SIGINT),
                                new UnixSignal (Mono.Unix.Native.Signum.SIGTERM),
                            };

                            Console.WriteLine("Press Ctrl-C to exit");
                            // block until a SIGINT or SIGTERM signal is generated.
                            int which = UnixSignal.WaitAny(signals, -1);

                            Console.WriteLine("Got a {0} signal, exiting", signals[which].Signum);
                        }


                    }
                }
            }

            return 0;

        }

    }
}
