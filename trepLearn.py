import trep
from trep import tx,ty,tz,rx,ry,rz
import trep.potentials
import time
import trep.visual as visual
import math
import numpy as np
import csv

# import states from .csv

# make sure .csv has ['motor1', 'motor2', 'motor3'] 
# or trep model's q = q = [z_com, motor1, center_theta1, center_theta2, motor2, right_theta, motor3, left_theta]

def getStates(filename):
    
    # read states from csv
    states = []
    with open(filename, 'r') as csvfile:
        csvReader = csv.reader(csvfile)
        for row in csvReader:
            float_row = []
            for state in row:
                float_row.append(float(state))
            states.append(float_row)
        
    # detect and print the format (3 motor angles or all trep angles)    
    print np.shape(states)
    if (np.shape(states)[1]==3):
        print 'Only motor torques'
    else:
        print 'States for trep model'
    
    return states

filename = 'JointVelocities.csv'
states = getStates(filename)

# print states

# OR calculate states with forward kinematics

def geomFK(qa,lengths,solOption):
    
    # Description:
    #    Computes the foot pose, using the actuated joint positions.
    #    Uses geometry (intersecting circles) to solve for the locations of the 
    #    "ankle" (xA,yA) and the "upper ankle" (xuA, yuA) relative to the base 
    #    frame, then calculates unactuated joint positions and the foot pose.

    #  References:
    #    https://math.stackexchange.com/questions/256100/how-can-i-find-the-points-at-which-two-circles-intersect
    #    http://mathworld.wolfram.com/Circle-CircleIntersection.html

    #  Inputs:
    #    qa:     actuated joint positions (column vector)
    #            qa = [theta1; phi1; psi1];
    #    lengths: the relevant dimensions of the robot, stored in a 10x1 column
    #        vector according to this order:
    #        lengths = [L1; L2; L3; L4; L5 L6; L7; L8; B1x; B2x; B1y; B2y], where
    #        B1x, B1y, L1, L2, and L8 correspond to the theta-chain,
    #        L3, L4, L7, and L8 correspond to the phi-chain, and
    #        B2x, B2y, L5, L6, and L8 correspond to the psi-chain.
    #    solOption: 1, 2, 3, or 4
    #        1: function uses (xA1,yA1) and (xuA1, yuA1)
    #        2: function uses (xA1,yA1) and (xuA2, yuA2)
    #        3: function uses (xA2,yA2) and (xuA1, yuA1)
    #        4: function uses (xA2,yA2) and (xuA2, yuA2)

    #  Outputs:
    #    qu:         6x1 column vector
    #        qu = [th2; th3; ph2; ph3; ps2; ps3];
    #    footPose:   3-element row vector which comprises:
    #        xF:     x-coordinate of the foot, relative to the body
    #        yF:     y-coordinate of the foot, relative to the body
    #        angF:   angle of the foot, relative to the body.

    #  Dependencies:
    #    none

    # "unpack" link lengths from "lengths" input vector:
    # Note: this step is just for cleaner-looking code.
    
    # make our own wrap to pi function
    def wrapToPi(phases):
        return ((phases+np.pi)%(2*np.pi)) - np.pi
        
    L1 = lengths[0]
    L2 = lengths[1]
    L3 = lengths[2]
    L4 = lengths[3]
    L5 = lengths[4]
    L6 = lengths[5]
    L7 = lengths[6]
    L8 = lengths[7]
    B1x = lengths[8]
    B2x = lengths[9]
    B1y = lengths[10]
    B2y = lengths[11]

    #  "unpack" actuated joint angles from "qa" input vector:
    #  Note: this step is just for cleaner-looking code.
    th1 = qa[0] # left
    ph1 = qa[1] # center
    ps1 = qa[2] # right
    
#     print 'Theta1 in degrees is %.3f' % ((th1*(180/np.pi)))
#     print 'Phi1 in degrees is %.3f' % ((ph1*(180/np.pi)))
#     print 'Psi1 in degrees is %.3f' % ((ps1*(180/np.pi)))

    # create empty arrays:
    qu = np.zeros((6,1))
    footPose = np.zeros((3,1))
    
    # initialize pi
    pi = math.pi

    # Calculate (xA, yA) using th1 and ps1:
    xkth = -B1x + L1*math.cos(th1)  #x-coordinate of theta-chain "knee"
    ykth = B1y + L1*math.sin(th1)   # y-coordinate of theta-chain "knee"

    xkps = B2x + L5*math.cos(ps1)  # x-coordinate of psi-chain "knee"
    ykps = B2y + L5*math.sin(ps1)  # y-coordinate of psi-chain "knee"

    a = math.sqrt((xkth - xkps)**2 + (ykth - ykps)**2)    # intermediate variable
    b = (L2**2 - L6**2 + a**2)/(2*a)                  # intermediate variable
    c = math.sqrt(L2**2 - b**2)                           # intermediate variable

    xA1 = (b/a)*(xkps - xkth) + (c/a)*(ykps - ykth) + xkth   # solution 1/2 xA-coordinate
    xA2 = (b/a)*(xkps - xkth) - (c/a)*(ykps - ykth) + xkth   # solution 3/4 xA-coordinate

    yA1 = (b/a)*(ykps - ykth) - (c/a)*(xkps - xkth) + ykth   # solution 1/2 yA-coordinate
    yA2 = (b/a)*(ykps - ykth) + (c/a)*(xkps - xkth) + ykth   # solution 3/4 yA-coordinate
    
    if ( (solOption==1) | (solOption==2) ):
        xA = xA1
        yA = yA1
    else:
        xA = xA2
        yA = yA2
    
    # fprintf('(xA, yA) = (%f, %f)\n',[xA, yA]);

    # Calculate (xuA, yuA) using ph1 and (xA, yA):
    xkph = L3*math.cos(ph1)  # x-coordinate of phi-chain "knee"
    ykph = L3*math.sin(ph1)   # y-coordinate of phi-chain "knee"
    
    d = math.sqrt((xkph - xA)**2 + (ykph - yA)**2)    # intermediate variable
    e = (L4**2 - L7**2 + d**2)/(2*d)                  # intermediate variable
    f = math.sqrt(L4**2 - e**2)                       # intermediate variable

    xuA1 = (e/d)*(xA - xkph) - (f/d)*(yA - ykph) + xkph   # solution 1/3 xuA-coordinate
    xuA2 = (e/d)*(xA - xkph) + (f/d)*(yA - ykph) + xkph   # solution 2/4 xuA-coordinate

    yuA1 = (e/d)*(yA - ykph) + (f/d)*(xA - xkph) + ykph   # solution 1/3 xuA-coordinate
    yuA2 = (e/d)*(yA - ykph) - (f/d)*(xA - xkph) + ykph   # solution 2/4 xuA-coordinate

    if ( (solOption==1) | (solOption==3) ):
        xuA = xuA1
        yuA = yuA1
    else:
        xuA = xuA2
        yuA = yuA2
    
    # fprintf('(xuA, yuA) = (%f, %f)\n',[xuA, yuA]);

    # Solve for "knee" angles (th2, ph2, ps2):

    # theta-chain--------------------------------------------------------------
    # Calculate (x,y) location of theta-chain "hip" joint:
    xHtheta = -B1x
    yHtheta = B1y

    # ankle relative to theta-hip:
    xAptheta = -xHtheta + xA
    yAptheta = yHtheta - yA

    # Calculate knee angle:
    betatheta = wrapToPi(math.acos((L1**2 + L2**2 - xAptheta**2 - yAptheta**2)/(2*L1*L2)))
    th2 = wrapToPi(pi - betatheta)

    # fprintf('theta2 = %f degrees\n',th2*(180/pi));

    # save th2 to qu array:
    qu[0] = th2

    # phi-chain----------------------------------------------------------------
    # Calculate knee angle:
    betaphi = wrapToPi(math.acos((L3**2 + L4**2 - xuA**2 - yuA**2)/(2*L3*L4)))
    ph2 = wrapToPi(pi - betaphi)

    # fprintf('phi2 = %f degrees\n',ph2*(180/pi));

    # save ph2 to qu array:
    qu[2] = ph2

    # psi-chain----------------------------------------------------------------
    # Calculate (x,y) location of "hip" joint:
    xHpsi = B2x
    yHpsi = B2y

    # Calculate hip angle:
    xAppsi = xHpsi - xA
    yAppsi = yHpsi - yA

    # Calculate knee angle:
    betapsi = wrapToPi(math.acos((L5**2 + L6**2 - xAppsi**2 - yAppsi**2)/(2*L5*L6)))
    ps2 = wrapToPi(pi + betapsi)

    # fprintf('psi2 = %f degrees\n',ps2*(180/pi));

    # save ps2 to qu array:
    qu[4] = ps2

    # Calculate the foot pose:
    footAngle = wrapToPi(pi+ math.atan2(yuA - yA, xuA - xA))
    footX = xA + L8*math.cos(footAngle)
    footY = yA + L8*math.sin(footAngle)

    # "pack" foot pose variables into footPose:
    footPose[0] = footX
    footPose[1] = footY
    footPose[2] = footAngle

    # Calculate the third angle in each open chain:
    th3 = footAngle - th1 - th2
    ph3 = footAngle - ph1 - ph2
    ps3 = footAngle - ps1 - ps2

    qu[1] = th3
    qu[3] = ph3
    qu[5] = ps3
    
    # qu = [th2, th3, ph2, ph3, ps2, ps3] = [left_theta, ~, center1, center2, right_theta, ~]
    qtrep = [0.1-footY, ph1+(np.pi/2), ph2, ph3, ps2, th2, ps1+(np.pi/2), th1+(np.pi/2)]
    
    return (qu, qtrep, footPose)

def subchainIK(footPose, lengths):
#     %% Description:ps1+(np.pi/2)
#     %   Computes inverse kinematics for each sub-chain (theta, phi, and psi
#     %   chains)
#     %
#     % Inputs:
#     %   footPose: a 3x1 column vector representing the foot pose:
#     %       footPose = [xF;
#     %                   yF;
#     %                   angF];
#     %   lengths: the relevant dimensions of the robot, stored in a 10x1 column
#     %       vector according to this order:
#     %       lengths = [L1; L2; L3; L4; L5 L6; L7; L8; B1; B2], where
#     %       B1, L1, L2, and L8 correspond to the theta-chain,
#     %       L3, L4, L7, and L8 correspond to the phi-chain, and
#     %       B2, L5, L6, and L8 correspond to the psi-chain.
#     %
#     % Outputs:
#     %   angles = [theta1    theta2  theta3;
#     %             phi1      phi2    phi3;
#     %             psi1      psi2    psi3];
#     %
#     % Dependencies:
#     %   none

#     %% "unpack" link lengths from "lengths" input vector:
#     % Note: this step is just for cleaner-looking code.

    # make our own wrap to pi function
    def wrapToPi(phases):
        return ((phases+np.pi)%(2*np.pi)) - np.pi


    L1 = lengths[0]
    L2 = lengths[1]
    L3 = lengths[2]
    L4 = lengths[3]
    L5 = lengths[4]
    L6 = lengths[5]
    L7 = lengths[6]
    L8 = lengths[7]
    B1x = lengths[8]
    B2x = lengths[9]
    B1y = lengths[10]
    B2y = lengths[11]

#     %% Extract foot pose:
    xF = footPose[0]
    yF = footPose[1]
    angF = footPose[2]

#     % Calculate (x,y) location of "lower ankle" joint, from foot pose:
    xA = xF - L8*math.cos(angF)
    yA = yF - L8*math.sin(angF)

#     % Calculate (x,y) location of "upper ankle" joint, from foot pose:
    xAu = xF - (L7+L8)*math.cos(angF)
    yAu = yF - (L7+L8)*math.sin(angF)

#     %% IK for theta-chain:
#     % Calculate (x,y) location of "hip" joint:
    xHtheta = -B1x
    yHtheta = B1y

#     % Calculate hip angle:
    xAptheta = -xHtheta + xA
    yAptheta = yHtheta - yA
    gammatheta = wrapToPi(abs(math.atan2(yAptheta,xAptheta)))
    alphatheta = wrapToPi(math.acos((xAptheta**2 + yAptheta**2 + L1**2 - L2**2)/(2*L1*math.sqrt(xAptheta**2 + yAptheta**2))))
    theta1 = wrapToPi(-gammatheta - alphatheta)

#     % Calculate (x,y) location of "knee" joint:
    xKtheta = xHtheta + L1*math.cos(theta1)
    yKtheta = yHtheta + L1*math.sin(theta1)

#     % Calculate knee angle:
    betatheta = wrapToPi(math.acos((L1**2 + L2**2 - xAptheta**2 - yAptheta**2)/(2*L1*L2)))
    theta2 = wrapToPi(np.pi - betatheta)

#     % Calculate (x,y,angle) of "lower ankle" joint, using FK:
    xAtheta = xKtheta + L2*math.cos(theta1 + theta2)
    yAtheta = yKtheta + L2*math.sin(theta1 + theta2)
    theta3 = wrapToPi(angF - theta1 - theta2)

#     %% IK for phi-chain:
#     % Calculate (x,y) location of "hip" joint:
    xHphi = 0
    yHphi = 0

#     % Calculate hip angle:
    gammaphi = wrapToPi(abs(math.atan2(yAu,xAu)))
    alphaphi = wrapToPi(math.acos((xAu**2 + yAu**2 + L3**2 - L4**2)/(2*L3*math.sqrt(xAu**2 + yAu**2))))
    phi1 = wrapToPi(-gammaphi - alphaphi)

#     % Calculate (x,y) location of "knee" joint:
    xKphi = xHphi + L3*math.cos(phi1)
    yKphi = yHphi + L3*math.sin(phi1)

#     % Calculate knee angle:
    betaphi = wrapToPi(math.acos((L3**2 + L4**2 - xAu**2 - yAu**2)/(2*L3*L4)))
    phi2 = wrapToPi(np.pi - betaphi)

#     % Calculate (x,y,angle) of "upper ankle" joint, using FK:
    xAuphi = xKphi + L4*math.cos(phi1 + phi2)
    yAuphi = yKphi + L4*math.sin(phi1 + phi2)
    phi3 = wrapToPi(angF - phi1 - phi2)

#     %% IK for psi-chain:
#     % Calculate (x,y) location of "hip" joint:
    xHpsi = B2x
    yHpsi = B2y

#     % Calculate hip angle:
    xAppsi = xHpsi - xA
    yAppsi = yHpsi - yA
    gammapsi = wrapToPi(abs(math.atan2(yAppsi,xAppsi)))
    alphapsi = wrapToPi(math.acos((xAppsi**2 + yAppsi**2 + L5**2 - L6**2)/(2*L5*math.sqrt(xAppsi**2 + yAppsi**2))))
    psi1 = wrapToPi(np.pi + gammapsi + alphapsi)

#     % Calculate (x,y) location of "knee" joint:
    xKpsi = xHpsi + L5*math.cos(psi1)
    yKpsi = yHpsi + L5*math.sin(psi1)

#     % Calculate knee angle:
    betapsi = wrapToPi(math.acos((L5**2 + L6**2 - xAppsi**2 - yAppsi**2)/(2*L5*L6)))
    psi2 = wrapToPi(np.pi + betapsi)

#     % Calculate (x,y,angle) of "lower ankle" joint, using FK:
    xApsi = xKpsi + L6*math.cos(psi1 + psi2)
    yApsi = yKpsi + L6*math.sin(psi1 + psi2)
    psi3 = wrapToPi(angF - psi1 - psi2)

#     %% Return joint angles:
    angles = [[theta1,theta2,theta3[0]],[phi1,phi2,phi3[0]],[psi1,psi2,psi3[0]]] # [motor1,motor2,motor3]
    
#     qtrep = [0.1-yF[0], phi1+(np.pi/2), phi2, phi3[0], psi2, theta2, psi1+(np.pi/2), theta1+(np.pi/2)] # if motor2 and motor3 are kinematic
    qtrep = [0.1-yF[0], phi1+(np.pi/2), phi2, phi3[0], psi1+(np.pi/2), psi2, theta1+(np.pi/2), theta2] # if not kinematic
    
    return (qtrep, angles)

# real link lengths, lengths = [L1; L2; L3; L4; L5 L6; L7; L8; B1x; B2x; B1y; B2y] (from google doc)
# OLD lengths = [0.053, 0.139, 0.097, 0.0983, 0.053, 0.139, 0.0692, 0.1018, 0.0573, 0.0573, 0.0082, 0.0082]
lengths = [0.0657, 0.1517, 0.0843, 0.0856, 0.0657, 0.1517, 0.0691, 0.1016, 0.0573, 0.0573, 0.0082, 0.0082]

# interpolated motor angles
# motor_angles = [qnew[1]-(np.pi/2), qnew[4]-(np.pi/2), qnew[6]-(np.pi/2)]
# motor_angles = [np.radians(-144.85), np.radians(-166.9), np.radians(-35.15)]
# (qU, qtrep, footPose) = geomFK(motor_angles, lengths, 1)
# print footPose
# angles = subchainIK(footPose, lengths)
# footPose[1] = -0.25
# print footPose
# final_angles = subchainIK(footPose, lengths)
# print final_angles
# footPose[1] = -0.18
# init_angles = subchainIK(footPose, lengths)
# print init_angles

# set up robot model in stance
hopper_stance = trep.System()
hopper_stance.import_frames([
    # center of mass
    tz('z_com',mass=0.001,name='CoM_robot'), [
        # middle links
        tz(-0.1), [
            rx('motor1',name='motor1'), [
                tz(-0.0843,name='center_link1'), [
                    rx('center_theta1'), [
                        tz(-0.0856,name='center_link2'), [
                            rx('center_theta2'), [
                                tz(-0.0691,mass=0,name='center_attach'), [
                                    tz(-0.1016,name='foot',mass=0)
                                ]
                            ]
                        ]
                    ]
                ]
            ]
        ],
        # right links
        ty(0.0573), [
            tz(-0.0918), [
                rx('motor2',name='motor2'), [
                    tz(-0.0657,name='right_link'), [
                        rx('right_theta'), [
                            tz(-0.1517,name='right_attach')
                        ]
                    ]
                ]
            ]
        ],
        # left links
        ty(-0.0573), [
            tz(-0.0918), [
                rx('motor3',name='motor3'), [
                    tz(-0.0657,name='left_link'), [
                        rx('left_theta'), [
                            tz(-0.1517,name='left_attach')
                        ]
                    ]
                ]
            ]
        ]
    ]
])

# NOTE: q = [z_com, motor1, center_theta1, center_theta2, motor2, right_theta, motor3, left_theta]

# Establish gravity
trep.potentials.Gravity(hopper_stance)
trep.forces.Damping(hopper_stance, 0.1)

# Input Torque
# trep.forces.ConfigForce(hopper_stance, 'motor1', 'motor1_torque')
trep.forces.ConfigForce(hopper_stance, 'motor2', 'motor2_torque')
trep.forces.ConfigForce(hopper_stance, 'motor3', 'motor3_torque')

# Add constraints
trep.constraints.PointToPoint2D(hopper_stance,'yz','right_attach','center_attach')
trep.constraints.PointToPoint2D(hopper_stance,'yz','left_attach','center_attach')
trep.constraints.PointToPoint2D(hopper_stance,'yz','World','foot')
# constrain foot angle to vertical
# constrain outside angles to be symmetric
# constrain all three angles to their real ranges

print hopper_stance.nQk

# set up robot model in flight
hopper_flight = trep.System()
hopper_flight.import_frames([
    # center of mass
    tz('z_com',mass=5,name='CoM_robot'), [
        # middle links
        tz(-0.1), [
            rx('motor1',name='motor1'), [
                tz(-0.0843,name='center_link1'), [
                    rx('center_theta1'), [
                        tz(-0.0856,name='center_link2'), [
                            rx('center_theta2'), [
                                tz(-0.0691,mass=0,name='center_attach'), [
                                    tz(-0.1016,name='foot',mass=0)
                                ]
                            ]
                        ]
                    ]
                ]
            ]
        ],
        # right links
        ty(0.0573), [
            tz(-0.0918), [
                rx('motor2',name='motor2',kinematic=True), [
                    tz(-0.0657,name='right_link'), [
                        rx('right_theta'), [
                            tz(-0.1517,name='right_attach')
                        ]
                    ]
                ]
            ]
        ],
        # left links
        ty(-0.0573), [
            tz(-0.0918), [
                rx('motor3',name='motor3',kinematic=True), [
                    tz(-0.0657,name='left_link'), [
                        rx('left_theta'), [
                            tz(-0.1517,name='left_attach')
                        ]
                    ]
                ]
            ]
        ]
    ]
])

# NOTE: q = [z_com, motor1, center_theta1, center_theta2, motor2, right_theta, motor3, left_theta]

# Establish gravity
trep.potentials.Gravity(hopper_flight)
# trep.forces.Damping(system, 0.1)

# Input Torque
# trep.forces.ConfigForce(hopper, 'motor1', 'motor1_torque')
# trep.forces.ConfigForce(hopper, 'motor2', 'motor2_torque')
# trep.forces.ConfigForce(hopper, 'motor3', 'motor3_torque')

# Add constraints
trep.constraints.PointToPoint2D(hopper_flight,'yz','right_attach','center_attach')
trep.constraints.PointToPoint2D(hopper_flight,'yz','left_attach','center_attach')
# trep.constraints.PointToPoint2D(hopper_flight,'yz','World','foot')
# constrain foot angle to vertical
# constrain outside angles to be symmetric
# constrain all three angles to their real ranges

# simulate forward with trajectory (only in stance)

# interpolate between initial and final motor angles
tview = np.linspace(0,1,100001)

# use inverse kinematics to find initial and final angles
# motor_angles = [np.radians(-144.85), np.radians(-166.9), np.radians(-35.15)]
# (qU, qtrep, footPose) = geomFK(motor_angles, lengths, 1)

footPose = np.asarray([[0], [-0.24], [-np.pi/2]])
(qtrep,angles) = subchainIK(footPose, lengths)

footPose[1] = -0.28
(qtrep_final,final_angles) = subchainIK(footPose, lengths)

footPose[1] = -0.2
(qtrep_init,init_angles) = subchainIK(footPose, lengths)

qtrep_view = np.repeat(qtrep_init, 100001, axis=0).reshape((8,100001)).T

motor_init = [init_angles[1][0]+(np.pi/2), init_angles[2][0]+(np.pi/2), init_angles[0][0]+(np.pi/2)]
motor_final = [final_angles[1][0]+(np.pi/2), final_angles[2][0]+(np.pi/2), final_angles[0][0]+(np.pi/2)]

# print qtrep_init

# print hopper.configs

# motor_angles = np.asarray([np.linspace(motor_init[0],motor_final[0],1001), np.linspace(motor_init[1],motor_final[1],1001), np.linspace(motor_init[2],motor_final[2],1001)]).T
motor_angles = np.asarray([np.linspace(motor_init[1],motor_final[1],100001), np.linspace(motor_init[2],motor_final[2],100001)]).T

# print motor_angles

footPose_Y = np.asarray([np.linspace(0,0,101), np.linspace(-0.18,-0.25,101), np.linspace((np.pi/2),(np.pi/2),101)]).T


# force setup
kt = 1 # motor torque constant
torque = 19*kt; # apply constant torque
click = 2 * (10**-6) # time unit for applying 


dt = 0.00001
tf = 1.0

def simulate_system(system, motor_angles):
    
    system.satisfy_constraints(tolerance=1e-1, keep_kinematic=True)
    q0 = system.q
    tcur = 0.0
    
    mvi = trep.MidpointVI(system)
    mvi.initialize_from_configs(tcur,q0,dt,q0)
    
    T = [mvi.t1]
    Q = [mvi.q1]
    steps = 0
    
    while mvi.t1 < tf:
        system.satisfy_constraints(tolerance=1e-1, keep_kinematic=True)
        mvi.step(mvi.t2+dt,tuple([0,0]),tuple(motor_angles[steps,:]),5000)
        T.append(mvi.t1)
        Q.append(mvi.q1)
        steps = steps+1
        
        # TODO: determine force necessary to make the change specified by the change in motor angles
        # TODO: make motor1 a kinematic variable, that can be determined from system dynamics?
        
    return (T,Q)
        
hopper_stance.q = qtrep_init

# simulate
print 'Simulating...'
(tsim, qsim) = simulate_system(hopper_stance, motor_angles)
print 'Done!'

# simulate forward with forcing (start in stance, move to flight)

# get initial motor angles for stance
tview = np.linspace(0,1,100001)

footPose = np.asarray([[0], [-0.24], [-np.pi/2]]) # valid initialization from previous cell

footPose[1] = -0.2
(qtrep_init,init_angles) = subchainIK(footPose, lengths) # angles for squatting pose

qtrep_view = np.repeat(qtrep_init, 100001, axis=0).reshape((8,100001)).T

motor_init = [init_angles[1][0]+(np.pi/2), init_angles[2][0]+(np.pi/2), init_angles[0][0]+(np.pi/2)]

# force setup
kt = 0.217 # motor torque constant from datasheet
torque = 19*kt; # apply constant torque
click = 2 * (10**-6) # time unit for applying high torques (100 clicks)

# inputs [motor2, motor3]
inputs = [[-5*kt, 5*kt], [-19*kt, 19*kt]] # torques for balancing and jumping

dt = 0.00001
tf = 0.2

def simulate_system(system):
    
    system.satisfy_constraints(tolerance=1e-1, keep_kinematic=True)
    q0 = system.q
    tcur = 0.0
    
    mvi = trep.MidpointVI(system)
    mvi.initialize_from_configs(tcur,q0,dt,q0)
    
    T = [mvi.t1]
    Q = [mvi.q1]
    steps = 0
    
    while mvi.t1 < tf:
#         system.satisfy_constraints(tolerance=1e-1, keep_kinematic=True)
#         if mvi.t1 < (400*click):
#             u = tuple([-19*kt, 19*kt])
#         else:
#             u = tuple([0,0])
          
        u = tuple([0*kt, 0*kt])
        mvi.step(mvi.t2+dt,u,(),5000)
        T.append(mvi.t1)
        Q.append(mvi.q1)
        steps = steps+1
        
    # while tcur < tf
        # stance for 100 clicks
        
        # flight
        
        # make sure tf is ~0.2 s
        
        
    return (T,Q)
        
hopper_stance.q = qtrep_init

# simulate
print 'Simulating...'
(tsim, qsim) = simulate_system(hopper_stance)
print 'Done!'

# visual.visualize_3d([ visual.VisualItem3D(hopper_stance, tview, qtrep_view) ], camera_pos = [(1,0,0)])
visual.visualize_3d([ visual.VisualItem3D(hopper_stance, tsim, qsim) ],camera_pos = [(1,0,0)])