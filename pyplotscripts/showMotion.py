import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

filepath = '/home/aayush/workspace/sandbox/VIROB/tmp/';

filename = '/home/aayush/workspace/sandbox/VIROB/LearnVIORB-master/Examples/Monocular/KeyFrameTrajectory.txt';

NS = np.loadtxt(filename);

time = NS[:,0];

fig = plt.figure(1);
ax = fig.add_subplot(111, projection='3d');
ax.plot(NS[:,1],NS[:,2],NS[:,3]);

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('Position in initial frame');
plt.savefig(filepath+'Pw.eps', format="eps");

plt.show();

#biasa = np.loadtxt(filepath+'biasa.txt');
#plt.figure(1);
#p11, =plt.plot(biasa[:,0]-biasa[0,0],biasa[:,1]);
#p12, =plt.plot(biasa[:,0]-biasa[0,0],biasa[:,2]);
#p13, =plt.plot(biasa[:,0]-biasa[0,0],biasa[:,3]);
#plt.title('bias-acc');
#plt.legend([p11,p12,p13],["x","y","z"]);
#plt.savefig(filepath+"biasa.eps", format="eps")
##plt.legend(p12,'y');
##plt.legend(p13,'z');

#scale = np.loadtxt(filepath+'scale.txt');
#plt.figure(2);
#[p21,p22] = plt.plot(scale[:,0]-scale[0,0],scale[:,1:3]);
#plt.title('scale');
#plt.legend([p21,p22],['aftopt','befopt']);
#plt.savefig(filepath+'/scale.eps', format="eps")

#condnum = np.loadtxt(filepath+'condnum.txt');
#plt.figure(3);
#plt.plot(condnum[:,0]-condnum[0,0],condnum[:,1]/condnum[:,6]);
#plt.title('condnum');
#plt.savefig(filepath+'condnum.eps', format="eps")


#biasg = np.loadtxt(filepath+'biasg.txt');
#plt.figure(4);
#p41, =plt.plot(biasg[:,0]-biasg[0,0],biasg[:,1]);
#p42, =plt.plot(biasg[:,0]-biasg[0,0],biasg[:,2]);
#p43, =plt.plot(biasg[:,0]-biasg[0,0],biasg[:,3]);
#plt.title('bias-gyr');
#plt.legend([p41,p42,p43],["x","y","z"]);
#plt.savefig(filepath+"biasg.eps", format="eps")

#plt.show();
