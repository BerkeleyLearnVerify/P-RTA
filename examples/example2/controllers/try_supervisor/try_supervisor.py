"""import dill
import socket
import sys

file_name = sys.argv[1]

sckt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sckt.connect(('127.0.0.1', 8888))
msg = sckt.recv(8192)
scene = dill.loads(msg)
msg = dill.dumps("OK")
sckt.send(msg)

robot = scene.objects[0].position
charger = scene.objects[1].position
slave1 = scene.objects[2].position
slave2 = scene.objects[3].position
slave3 = (-slave1[0], -slave1[1])
slave4 = (-slave2[0], -slave2[1])
box1 = scene.objects[4].position
box2 = scene.objects[5].position
box3 = scene.objects[6].position
box4 = (-box1[0], -box1[1])
box5 = (-box2[0], -box2[1])
box6 = (-box3[0], -box3[1])

f = open("../../worlds/example2.wbt", "r")
lines = f.readlines()
f.close()

lines[61] = '  translation ' + str(robot[0]) + ' 0 ' + str(robot[1]) + '\n'
lines[241] = '  translation ' + str(charger[0]) + ' 0 ' + str(charger[1]) + '\n'

lines[303] = '  translation ' + str(slave1[0]) + ' 0 ' + str(slave1[1]) + '\n'
lines[303] = '  translation ' + str(slave2[0]) + ' 0 ' + str(slave2[1]) + '\n'
lines[635] = '  translation ' + str(slave3[0]) + ' 0 ' + str(slave3[1]) + '\n'
lines[801] = '  translation ' + str(slave4[0]) + ' 0 ' + str(slave4[1]) + '\n'


lines[19] = '  translation ' + str(box1[0]) + ' 0.5 ' + str(box1[1]) + '\n'
lines[26] = '  translation ' + str(box2[0]) + ' 0.5 ' + str(box2[1]) + '\n'
lines[33] = '  translation ' + str(box3[0]) + ' 0.5 ' + str(box3[1]) + '\n'
lines[40] = '  translation ' + str(box4[0]) + ' 0.5 ' + str(box4[1]) + '\n'
lines[47] = '  translation ' + str(box5[0]) + ' 0.5 ' + str(box5[1]) + '\n'
lines[54] = '  translation ' + str(box6[0]) + ' 0.5 ' + str(box6[1]) + '\n'

f = open("../../worlds/" + file_name, "w+")
f.writelines(lines)
f.close()

scene = (str(robot[0]) + "\n" + str(robot[1]) + "\n" +
         str(charger[0]) + "\n" + str(charger[1]) + "\n" +
         str(box1[0]) + "\n" + str(box1[1]) + "\n" +
         str(box2[0]) + "\n" + str(box2[1]) + "\n" +
         str(box3[0]) + "\n" + str(box3[1]) + "\n" +
         str(box4[0]) + "\n" + str(box4[1]) + "\n" +
         str(box5[0]) + "\n" + str(box5[1]) + "\n" +
         str(box6[0]) + "\n" + str(box6[1]) + "\n")

f = open("scene", "w+")
f.write(scene)
f.close()

"""

from verifai.simulators.webots.webots_task import webots_task
from verifai.simulators.webots.client_webots import ClientWebots
from time import sleep
try:
    from controller import Supervisor
except ModuleNotFoundError:
    import sys
    sys.exit('This functionality requires webots to be installed')

from dotmap import DotMap
import numpy as np

# Defining the task as a webots task
class scenic_cones(webots_task):
    def __init__(self, N_SIM_STEPS, supervisor):
        super().__init__(N_SIM_STEPS, supervisor)
        self.traj_x = np.zeros(self.N_SIM_STEPS)
        self.traj_z = np.zeros(self.N_SIM_STEPS)
        self.traj_battery = np.zeros(self.N_SIM_STEPS)

    def use_sample(self, sample):
        print('Sample recieved:', sample)

        robot = self.supervisor.getFromDef('robot')
        charger = self.supervisor.getFromDef('charger')
        slave1 = self.supervisor.getFromDef('slave1')
        slave2 = self.supervisor.getFromDef('slave2')
        slave3 = self.supervisor.getFromDef('slave3')
        slave4 = self.supervisor.getFromDef('slave4')
        box1 = self.supervisor.getFromDef('box1')
        box2 = self.supervisor.getFromDef('box2')
        box3 = self.supervisor.getFromDef('box3')
        box4 = self.supervisor.getFromDef('box4')
        box5 = self.supervisor.getFromDef('box5')
        box6 = self.supervisor.getFromDef('box6')

        robot_old_pos = self.supervisor.getFromDef('robot').getField('translation').getSFVec3f()
        robot_pos = tuple(map(lambda x: round(x, 3), sample.objects.robot))
        robot.getField('translation').setSFVec3f([robot_pos[0], robot_old_pos[1], robot_pos[1]])
        
        charger_old_pos = self.supervisor.getFromDef('charger').getField('translation').getSFVec3f()
        charger_pos = tuple(map(lambda x: round(x, 3), sample.objects.charger))
        charger.getField('translation').setSFVec3f([charger_pos[0], charger_old_pos[1], charger_pos[1]])

        slave1_old_pos = self.supervisor.getFromDef('slave1').getField('translation').getSFVec3f()
        slave1_pos = tuple(map(lambda x: round(x, 3), sample.objects.slave1))
        slave1.getField('translation').setSFVec3f([slave1_pos[0], slave1_old_pos[1], slave1_pos[1]])
        
        slave2_old_pos = self.supervisor.getFromDef('slave2').getField('translation').getSFVec3f()
        slave2_pos = tuple(map(lambda x: round(x, 3), sample.objects.slave2))
        slave2.getField('translation').setSFVec3f([slave2_pos[0], slave2_old_pos[1], slave2_pos[1]])
        
        slave3_old_pos = self.supervisor.getFromDef('slave3').getField('translation').getSFVec3f()
        slave3_pos = tuple(map(lambda x: round(x, 3), sample.objects.slave3))
        slave3.getField('translation').setSFVec3f([slave3_pos[0], slave3_old_pos[1], slave3_pos[1]])
        
        slave4_old_pos = self.supervisor.getFromDef('slave4').getField('translation').getSFVec3f()
        slave4_pos = tuple(map(lambda x: round(x, 3), sample.objects.slave4))
        slave4.getField('translation').setSFVec3f([slave4_pos[0], slave4_old_pos[1], slave4_pos[1]])
        
        box1_old_pos = self.supervisor.getFromDef('box1').getField('translation').getSFVec3f()
        box1_pos = tuple(map(lambda x: round(x, 3), sample.objects.box1))
        box1.getField('translation').setSFVec3f([box1_pos[0], box1_old_pos[1], box1_pos[1]])
        
        box2_old_pos = self.supervisor.getFromDef('box2').getField('translation').getSFVec3f()
        box2_pos = tuple(map(lambda x: round(x, 3), sample.objects.box2))
        box2.getField('translation').setSFVec3f([box2_pos[0], box2_old_pos[1], box2_pos[1]])
        
        box3_old_pos = self.supervisor.getFromDef('box3').getField('translation').getSFVec3f()
        box3_pos = tuple(map(lambda x: round(x, 3), sample.objects.box3))
        box3.getField('translation').setSFVec3f([box3_pos[0], box3_old_pos[1], box3_pos[1]])
        
        box4_old_pos = self.supervisor.getFromDef('box4').getField('translation').getSFVec3f()
        box4_pos = tuple(map(lambda x: round(x, 3), sample.objects.box4))
        box4.getField('translation').setSFVec3f([box4_pos[0], box4_old_pos[1], box4_pos[1]])
        
        box5_old_pos = self.supervisor.getFromDef('box5').getField('translation').getSFVec3f()
        box5_pos = tuple(map(lambda x: round(x, 3), sample.objects.box5))
        box5.getField('translation').setSFVec3f([box5_pos[0], box5_old_pos[1], box5_pos[1]])
        
        box6_old_pos = self.supervisor.getFromDef('box6').getField('translation').getSFVec3f()
        box6_pos = tuple(map(lambda x: round(x, 3), sample.objects.box6))
        box6.getField('translation').setSFVec3f([box6_pos[0], box6_old_pos[1], box6_pos[1]])
        
        scene = (str(robot_pos[0]) + "\n" + str(robot_pos[1]) + "\n" +
                 str(charger_pos[0]) + "\n" + str(charger_pos[1]) + "\n" +
                 str(box1_pos[0]) + "\n" + str(box1_pos[1]) + "\n" +
                 str(box2_pos[0]) + "\n" + str(box2_pos[1]) + "\n" +
                 str(box3_pos[0]) + "\n" + str(box3_pos[1]) + "\n" +
                 str(box4_pos[0]) + "\n" + str(box4_pos[1]) + "\n" +
                 str(box5_pos[0]) + "\n" + str(box5_pos[1]) + "\n" +
                 str(box6_pos[0]) + "\n" + str(box6_pos[1]) + "\n")

        f = open("scene", "w+")
        f.write(scene)
        f.close()

        return robot, charger



    def run_task(self, sample):
        robot, charger = self.use_sample(sample)
        charger_pos = charger.getField("translation").getSFVec3f()
        for i in range(self.N_SIM_STEPS):
            self.supervisor.step(100)
            robot_pos = robot.getField("translation").getSFVec3f()
            battery_level = robot.getField("battery").getMFFloat(0)
            if (np.sqrt((robot_pos[0] - charger_pos[0])**2 + (robot_pos[2] - charger_pos[2])**2) <= 0.20):
                battery_level = 250;
                robot.getField("battery").setMFFloat(0, battery_level)
            self.traj_x[i] = robot_pos[0]
            self.traj_z[i] = robot_pos[2]
            self.traj_battery[i] = battery_level
        traj = {}
        traj['battery'] = [(j*0.016, b) for j, b in enumerate(self.traj_battery)]
        traj['geofence1'] = [(j*0.016, -250) if pos[0] > -0.2 and pos[0] < 0.2 and pos[1] > -1.2 and pos[1] < -0.8 else (j*0.016, 250) for j, pos in enumerate(zip(self.traj_x, self.traj_z))]
        traj['geofence2'] = [(j*0.016, -250) if pos[0] > -0.2 and pos[0] < 0.2 and pos[1] > 0.8 and pos[1] < 1.2 else (j*0.016, 250) for j, pos in enumerate(zip(self.traj_x, self.traj_z))]
        sim_results = traj
        return sim_results



PORT = 8888
BUFSIZE = 4096
N_SIM_STEPS = 3000
supervisor = Supervisor()
simulation_data = DotMap()
simulation_data.port = PORT
simulation_data.bufsize = BUFSIZE
simulation_data.task = scenic_cones(N_SIM_STEPS=N_SIM_STEPS, supervisor=supervisor)
client_task = ClientWebots(simulation_data)
if not client_task.run_client():
    print("End of scene generation")
    supervisor.simulationQuit(True)
