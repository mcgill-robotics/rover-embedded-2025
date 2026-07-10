
def test1():

    '''
    This test is to verify that on startup if the diretion is not changed, the the commands are discarded and the motor starts to the original setpoint
    as instructed

    Result: Frequency is slow enough for the motor to change setpoint once, so speed should be 1300
    '''

    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(1) # enough time to initially ramp up
    drive.run_motor(NodeID.RF_DRIVE, 1300)
    time.sleep(1)
    drive.run_motor(NodeID.RF_DRIVE, 1100)
    time.sleep(1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(1)


def test2():

    '''
    This test is to verify that on startup if the diretion is not changed, the the commands are discarded and the motor starts to the original setpoint
    as instructed --> changed frequency

    Result: Frequency is too fast for the motor to change setpoint, so speed should be 1500
    '''

    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(.5) # enough time to initially ramp up
    drive.run_motor(NodeID.RF_DRIVE, 1300)
    time.sleep(.5)
    drive.run_motor(NodeID.RF_DRIVE, 1100)
    time.sleep(.5)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(.5)

def test3():

    '''
    This test is to verify that on startup if the speed becomes 0 on startup, the motor will exit the start condition.

    Result: Motor stops in startup
    '''

    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(1) # enough time to initially ramp up
    drive.run_motor(NodeID.RF_DRIVE, 1300)
    time.sleep(1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(1)

def test4():

    '''
    This test is to verify that on startup if the speed becomes 0 on startup, the motor will exit the start condition.
    --> Faster frequency

    Result: Motor stops in startup
    '''

    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.5) # enough time to initially ramp up
    drive.run_motor(NodeID.RF_DRIVE, 1300)
    time.sleep(0.5)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.5)

def test5():

    '''
    This test is to verify that on startup if the speed becomes inverse direction on startup, the motor will exit the start condition.
    --> Faster frequency

    Result: Motor stops in startup
    '''

    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.5) # enough time to initially ramp up
    drive.run_motor(NodeID.RF_DRIVE, 1300)
    time.sleep(0.5)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.5)

def test6():

    '''
    This test is to verify that when in open loop transfoer to close loop, if the speed setpoint is changed nothing occurs

    Result: Motor stops in startup
    '''

    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(2.55) # enough time to be in teh open loop phase
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.5)


def test7():

    '''
    This test is to validate that the motor can respond to commands at the rate that they will actually be sent (10Hz) for may comp without startup errors
    '''
    for i in range(3):
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) # enough time to initially ramp up
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1) # enough time to initially ramp up
        drive.run_motor(NodeID.RF_DRIVE, 2100)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2500)
        time.sleep(0.1) # enough time to initially ramp up
        drive.run_motor(NodeID.RF_DRIVE, 2500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2400)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2400)
        time.sleep(0.1)
        #1 second
    
    time.sleep(2)
    drive.stop_motor(NodeID.RF_DRIVE)

def test8():

    '''
    This test is to validate that the motor can respond to commands at the rate that they will actually be sent (10Hz) for may comp with minor fluctuations in direction
    '''

    #this will all happen in the startstate (except last 3 calls of last iteration)
    for i in range (3):
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1560)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1400)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)

    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 800)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1400)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1)
    #1 second
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 800)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1400)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1)
    #1 second

    drive.read_speed(NodeID.RF_DRIVE)
    station.recv_msg(0.050)
    
    time.sleep(2)
    drive.stop_motor(NodeID.RF_DRIVE)

def test9():

    '''
    This test is to validate that the motor can respond to commands at the rate that they will actually be sent (10Hz) for may comp with major fluctuations in direction
    '''

    #this will all happen in the startstate (except last 3 calls of last iteration)
    for i in range (3):
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1560)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1400)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)

    for i in range(3):
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 900)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1)
        #1 second
        drive.run_motor(NodeID.RF_DRIVE, 1000)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1900)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        #1 second

    drive.read_speed(NodeID.RF_DRIVE)
    station.recv_msg(0.050)
    
    time.sleep(2)
    drive.stop_motor(NodeID.RF_DRIVE)

def test10():

    '''
    This test is to validate that the motor can respond to commands at the rate that they will actually be sent (10Hz) with many changes
    '''

    #this will all happen in the startstate (except last 3 calls of last iteration)
    for i in range (3):
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1560)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1400)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)

    # ramp down and try and start again, but stop
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    #1 second
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1600)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1700)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    #should hear it attempt to start twice but stop

    #1 second

    # ramp up again, and then make back to 0

    for i in range (3):
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1560)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1400)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)

        # ramp down and try and start again, but stop
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 900)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 0)
        time.sleep(0.1)
        #1 second

    drive.read_speed(NodeID.RF_DRIVE)
    station.recv_msg(0.05)
    
    time.sleep(2)
    drive.stop_motor(NodeID.RF_DRIVE)


def test11():

    '''
    This test is to validate that the motor can respond to commands at the rate that they will actually be sent (10Hz) with many stops and starts
    '''

    #this will all happen in the startstate (except last 3 calls of last iteration)
    for i in range (3):
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1560)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1400)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)

    # ramp down and try and start again, but stop
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    #1 second
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 10)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    #should hear it attempt to start twice but stop

    #1 second

    # ramp up again, and then make back to 0

    for i in range (5):
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1560)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1400)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)

     # ramp down and try and start again, but stop
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    #1 second
    #1 second

    drive.read_speed(NodeID.RF_DRIVE)
    station.recv_msg(0.05)
    
    time.sleep(2)

def test12():

    '''
    This test is to validate that the motor can respond to commands at the rate that they will actually be sent (10Hz) with chaning direction commands
    '''

    #this will all happen in the startstate (except last 3 calls of last iteration)
    for i in range (3):
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1560)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1400)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)

    # ramp down and try and start again, but stop
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1330)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    #1 second

    drive.read_speed(NodeID.RF_DRIVE)
    station.recv_msg(0.05)
    
    time.sleep(2)
    drive.stop_motor(NodeID.RF_DRIVE)

def test13():

    '''
    This test is to validate that the motor can respond to commands at the rate that they will actually be sent (10Hz) with chaning direction commands after stopping
    '''

    #this will all happen in the startstate (except last 3 calls of last iteration)
    for i in range (3):
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1560)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1400)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)

    # ramp down and try and start again, but stop
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1300)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1400)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1600)
    time.sleep(0.1)
    #1 second
    # ramp down and try and start again, but stop
    drive.run_motor(NodeID.RF_DRIVE, -1600)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1400)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1300)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    #1 second
    drive.run_motor(NodeID.RF_DRIVE, -1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1330)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    #1 second
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1600)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1700)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 0)
    time.sleep(0.1)

    #1 second

    drive.read_speed(NodeID.RF_DRIVE)
    station.recv_msg(0.05)
    
    time.sleep(2)
    # drive.stop_motor(NodeID.RF_DRIVE)

def test14():

    '''
    This test is to validate that the motor can respond to commands at the rate that they will actually be sent (10Hz) with chaning direction commands after stopping
    '''

    #this will all happen in the startstate (except last 3 calls of last iteration)
    for i in range (3):
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 2000)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1700)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1560)
        time.sleep(0.1) 
        drive.run_motor(NodeID.RF_DRIVE, 1400)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1500)
        time.sleep(0.1)
        drive.run_motor(NodeID.RF_DRIVE, 1600)
        time.sleep(0.1)

    # ramp down and try and start again, but stop
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1300)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1400)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1600)
    time.sleep(0.1)
    #1 second
    # ramp down and try and start again, but stop
    drive.run_motor(NodeID.RF_DRIVE, -1600)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1500)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1400)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1300)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    #1 second
    drive.run_motor(NodeID.RF_DRIVE, -1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1330)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    #1 second
    
    drive.run_motor(NodeID.RF_DRIVE, -1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1330)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    #1 second
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1330)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    #1 second
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1330)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    #1 second
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1330)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    #1 second
    drive.run_motor(NodeID.RF_DRIVE, 1500)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 900)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1330)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, 1200)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 100)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, 1000)
    time.sleep(0.1) 
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    drive.run_motor(NodeID.RF_DRIVE, -1000)
    time.sleep(0.1)
    #1 second

    drive.read_speed(NodeID.RF_DRIVE)
    station.recv_msg(0.05)
    
    time.sleep(2)

