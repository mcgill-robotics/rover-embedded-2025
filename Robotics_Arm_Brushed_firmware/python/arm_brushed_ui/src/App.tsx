import { useCallback, useEffect, useRef, useState } from 'react'
import './App.css'
import { Badge, Button, Card, Flex, Kbd, Text, TextField } from '@radix-ui/themes'

function App() {
  const [angle, setAngle] = useState("")
  const [connecting, setConnecting] = useState(false)
  const [ports, setPorts] = useState([])
  const lastHeartbeatTimeout = useRef<number|null>(null)
  const [server, setServer] = useState(location.host)
  const [serial, setSerial] = useState<string|null>(null)
  const [kbdControl, setKbdControl] = useState(true)
  const ws = useRef<WebSocket|null>(null)
  const [ serverConnected, setServerConnected ] = useState(false)
  const [ serialConnected, setSerialConnected ] = useState(false)
  const [lastCommand, setLastCommand] = useState<string|null>(null)
  const HEARTBEAT_INTERVAL = 3000
  const HEARTBEAT_TIMEOUT = 2000
  const [isScience, setIsScience] = useState(false)
  const commandMap = new Map(Object.entries({
    "c":"Gripper Open",
    "o":"Gripper Close",
    "w":"Roll CCW",
    "d":"Roll CW",
    "r":"Roll Stop",
    "s":"Gripper Stop",
    "p":"Pitch angle",
    "u":"Pitch Up",
    "i":"Pitch Down",
    "a":"Pitch Stop"
  }))

  const portButtons = ports.map(p => 
    <Button
    variant="outline"
     onClick={()=>connectSerial(p)}>{p}</Button>
  )

  const getPorts = async () => {
    let response = await fetch(`http://${server}/api/ports`)
    if (response.status == 200 ){
      let data = await response.json()
      let portList = data["ports"]
      if (portList.length === 0){
        setSerialConnected(false)
      }
      setPorts(portList)
    }
  }

  const connectSerial = (port:string) => {
    setLastCommand(null)
    sendMessage({
      "port":port,
      "auto":false
    })
  }

  const connectSerialAuto = () => {
    setLastCommand(null)
    sendMessage({
      "port":null,
      "auto":true
    })
  }

  const processKbdInput = useCallback((event:KeyboardEvent) => {
      if (kbdControl){
        let key = event.key.toLowerCase()
        if (key === "w"){
          //open gripper
          sendCommand('c')
        } else if (key == "s"){
          // close gripper
          sendCommand('o')
        }else if (key == "a"){
          // counter-clockwise roll
          sendCommand('w')
        }else if (key == "d"){
          // clockwise roll
          sendCommand('d')
        } else if (key == "q"){
          // roll stop
          sendCommand("r")
        }else if (key == "e"){
          // gripper stop
          sendCommand("s")
        }
        else if (key == "z"){
          // Pitch Up
          sendCommand("u")
        }
        else if (key == "x"){
          // Pitch Stop
          sendCommand("a")
        }
        else if (key == "c"){
          // Pitch Down
          sendCommand("i")
        }
      }
    }, [kbdControl, ws])


  useEffect(()=> {
    window.addEventListener("keydown", processKbdInput);
    return () => {
      window.removeEventListener("keydown", processKbdInput);
    }
  },[processKbdInput])

  const heartbeat = ()=>{
    let socket = ws.current;
    if (socket !== null){
      socket.send(JSON.stringify({conn:"HEARTBEAT"}))
      setTimeout(heartbeat, HEARTBEAT_INTERVAL)
    }
  }

  const heartbeatFailed = () => {
    let socket = ws.current;
    if (socket !== null){
      disconnectServer()
    }
  }

  const connectToServer = () => {
    setConnecting(true)
    ws.current = new WebSocket(`ws://${server}/ws`);
    let socket = ws.current;
    if (socket !== null){
      socket.onerror = function (_event) {
        setConnecting(false)
      }
      socket.onopen = function(_event){
        setConnecting(false)
        setServerConnected(true)
        heartbeat()
        getPorts().then(()=>{})
      }
      socket.onclose = function(_event){
        disconnectServer()
        if (lastHeartbeatTimeout.current !== null){
          clearTimeout(lastHeartbeatTimeout.current)
          lastHeartbeatTimeout.current = null
        }
      }
      socket.onmessage = function(event) {
        if (lastHeartbeatTimeout.current !== null){
          clearTimeout(lastHeartbeatTimeout.current)
        }
        lastHeartbeatTimeout.current = setTimeout(heartbeatFailed, HEARTBEAT_INTERVAL+HEARTBEAT_TIMEOUT)
        let message = event.data
        console.log(message)
        if (message !== null && message !== undefined){
          let data = JSON.parse(message)
          if (data["conn"] !== undefined && data["conn"] !== null) {
            if (data["conn"].trim() === "HEARTBEAT"){
              // ignore as we reset on any received message not just heartbeat
            } 
          } else if (data["serial"] !== undefined && data["serial"] !== null) { 
            if (data["serial"].trim()==="SERIAL_DISCONNECT"){
              getPorts().then(()=>{})
              setSerialConnected(false)
            } else if (data["serial"].trim()==="SERIAL_CONNECT"){
              setSerial(data["port"])
              console.log(`Serial port ${data["port"]}`)
              setSerialConnected(true)
            }
          }
        }
        
      };
    }
  }

  useEffect(()=>{
    if (!serialConnected){
      setLastCommand(null)
    }
  }, [serialConnected])

  const disconnectServer = () => {
      ws.current?.close()
      ws.current = null
      setSerial("")
      setSerialConnected(false)
      setServerConnected(false)
  }

  useEffect(()=>{
    getPorts().then(()=>{})
    connectToServer()
    return () => {
      disconnectServer()
    }
  }, []) // <-- empty dependency array

  function onInput(event:any){
    if (event.target.value !== null){
      setAngle(event.target.value)
    }
  }

  function onServerInput(event:any){
    if (event.target.value !== null){
      setServer(event.target.value)
    }
  }

  function updateLastMessage(value:string){
    value = value.trim()
    if (value.length > 0){
      let first = value.charAt(0)
      let msg = commandMap.get(first)
      if (msg !== undefined){
        if (first === "p"){
          msg = `${msg} set to ${value.slice(1)}`
        }
        setLastCommand(msg)
      }
    }
    
  }

  function sendCommand(value:string) {
    if (sendMessage({command:value.trim()})){
      updateLastMessage(value)
    }
  }

  function sendMessage(value:object) {
    let socket = ws.current;
    if (socket !== null){
      console.log(`Sending: ${JSON.stringify(value)}`)
      socket.send(JSON.stringify(value))      
      return true;
    }
    return false
  }

  return (
    <>
    <Flex p="5" direction="column" gap="5">
      <Flex gap="2" alignSelf="start" px="2" flexGrow="1">
        <Text align="left" size="6" weight="bold">Arm Brushed Control</Text>
        <Button 
          color="red"
          variant="soft"
          onClick={() => setIsScience(!isScience)}>
          {isScience ? "Switch to Arm" : "Switch to Science"}
        </Button>
      </Flex>
      <Card variant="classic">
        <Flex p="2" direction="column" gap="3">
          <Text size="5" weight="bold">Connection Status</Text>
          <Flex direction="row" gap="3">
            <Badge color={serverConnected? "green": "red"}>{serverConnected? `Server Connected (${server})`: "Server Disconnected"}</Badge>
            <Badge color={serialConnected? "green": "red"}>{serialConnected? `Board Connected (${serial})`: "Board Disconnected"}</Badge>
          </Flex>
          <Text weight="medium">Server:</Text>
          <TextField.Root color="gray" variant="soft" disabled={serverConnected || connecting} size="2" placeholder="Server URL..." onInput={onServerInput} defaultValue={server}/>
          {!serverConnected ? 
            <>
              <Button 
                disabled={connecting}
                onClick={connectToServer}>
                {connecting? "Connecting...": "Connect to server"}
              </Button>
            </>
             : <>
              <Button 
                  color="red"
                  onClick={disconnectServer}>
                  Disconnect
                </Button>
                <Text weight="medium">Serial Ports:</Text>
                {portButtons.length==0 ? "No port found":
                  <Flex direction="row" gap="3">
                    <Button
                      onClick={()=>connectSerialAuto()}>Auto</Button>
                    {portButtons}
                  </Flex>
                }
                <Button 
                  variant="soft"
                  onClick={getPorts}>
                  Refresh Serial Port List
                </Button>
                {lastCommand!==null?
                  <Text size="2">Last command: {lastCommand}</Text>
                  : <></>
                }
             </>}
          
        </Flex>
      </Card>
     
      <Card variant="classic">
        <Flex p="2" direction="column" gap="3" wrap="wrap">
          <Text size="5" weight="bold">Control Scheme</Text>
          <Flex direction="row" gap="2">
            <Flex direction="row" align="center" gap="1">
              <Text size="1">{isScience ? "Auger Up" : "Roll CCW"}: </Text>
              <Kbd variant="soft">a</Kbd>
            </Flex>
            <Flex direction="row" align="center" gap="1">
              <Text size="1">{isScience ? "Auger Down" : "Roll CW"}: </Text>
              <Kbd variant="soft">d</Kbd>
            </Flex>
            <Flex direction="row" align="center" gap="1">
              <Text size="1">{isScience ? "Lead Screw Right" : "Gripper Open"}: </Text>
              <Kbd variant="soft">w</Kbd>
            </Flex>
            <Flex direction="row" align="center" gap="1">
              <Text size="1">{isScience ? "Lead Screw Left" : "Gripper Close"}: </Text>
              <Kbd variant="soft">s</Kbd>
            </Flex>
            <Flex direction="row" align="center" gap="1">
              <Text size="1">{isScience ? "Auger Stop" : "Roll Stop"}: </Text>
              <Kbd variant="soft">q</Kbd>
            </Flex>
            <Flex direction="row" align="center" gap="1">
              <Text size="1">{isScience ? "Lead Screw Stop" : "Gripper Stop"}: </Text>
              <Kbd variant="soft">e</Kbd>
            </Flex>
            <Flex direction="row" align="center" gap="1">
              <Text size="1">{isScience ? "Drill down" : "Pitch Up"}: </Text>
              <Kbd variant="soft">z</Kbd>
            </Flex>
            <Flex direction="row" align="center" gap="1">
              <Text size="1">{isScience ? "Drill Up" : "Pitch Down"}: </Text>
              <Kbd variant="soft">c</Kbd>
            </Flex>
            <Flex direction="row" align="center" gap="1">
              <Text size="1">{isScience ? "Drill Stop" : "Pitch Stop"}: </Text>
              <Kbd variant="soft">x</Kbd>
            </Flex>
          </Flex>
          <Button 
            size="2"
            color = {kbdControl ? "red": undefined}
            onClick={() => {setKbdControl(!kbdControl)}}>
            {kbdControl ? "Disable Keyboard": "Enable Keyboard"}
          </Button>
        </Flex>
      </Card>
      {/* Pitch */}
      <Card variant="classic">
        <Flex p="2" direction="column" gap="3">
          <Text size="5" weight="bold">{isScience ? "Drill turn" : "Wrist Pitch"}</Text>
          {/* PID toggle */}
          <Flex direction="row" align="center" gap="3">
            <Button onClick={()=>{sendCommand("l")}}>Enable PID</Button>
            <Button color="amber" onClick={()=>{sendCommand("m")}}>Disable PID</Button>
          </Flex>
          <Text size="1" weight="bold">PID control:</Text>
          <Flex direction="row" align="center" gap="3">
            <TextField.Root color="gray" variant="soft" size="2" placeholder="Angle..." onInput={onInput}/>
            <Button variant="soft" onClick={()=>{sendCommand(`p${angle}`)}}>Send</Button>
            {/* <Button color="red" onClick={()=>{sendMessage('c')}}>Stop</Button> */}
          </Flex>
          <Text size="1" weight="bold">Non-PID control:</Text>
          <Flex direction="row" gap="2">
            <Button variant="soft" onClick={()=>{sendCommand('u')}}>{isScience ? "Drill Down" : "Up"}</Button>
            <Button variant="soft" onClick={()=>{sendCommand('i')}} >{isScience ? "Empty" : "Down"}</Button>
            <Button color="red" onClick={()=>{sendCommand('a')}}>Stop</Button>
          </Flex>
        </Flex>
      </Card>
      {/* Gripper control */}
    
      <Card variant="classic">
        <Flex p="2" direction="column" gap="3">
          <Text size="5" weight="bold">{isScience ? "Lead Screw" : "Gripper"}</Text>
          <Flex direction="row" gap="2">
            <Button variant="soft" onClick={()=>{sendCommand('c')}}>{isScience ? "Rover Right" : "Open"}</Button>
            <Button variant="soft" onClick={()=>{sendCommand('o')}} >{isScience ? "Rover Left" : "Close"}</Button>
            <Button color="red" onClick={()=>{sendCommand('s')}}>Stop</Button>
          </Flex>
        </Flex>
      </Card>
      {/* Roll */}
      <Card variant="classic">
        <Flex p="2" direction="column" gap="3">
          <Text size="5" weight="bold">{isScience ? "Drill Up/Down" : "Wrist Roll"}</Text>
          <Flex direction="row" gap="2">
            <Button variant="soft" onClick={()=>{sendCommand('d')}}>{isScience ? "Auger Up" : "Clockwise"}</Button>
            <Button variant="soft" onClick={()=>{sendCommand('w')}}>{isScience ? "Auger Down" : "Counter-Clockwise"}</Button>
            <Button color="red" onClick={()=>{sendCommand('r')}}>Stop</Button>
          </Flex>
        </Flex>
      </Card>
    </Flex>
    </>
  )
}

export default App
