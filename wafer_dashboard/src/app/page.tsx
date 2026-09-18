"use client";

import React, { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';
import { Joystick } from 'react-joystick-component';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Box, Cylinder } from '@react-three/drei';
import { Activity, Power, Wifi, ShieldAlert } from 'lucide-react';

const ROS_URL = 'ws://localhost:9090';

// Simple Mock 3D SCARA Robot
function ScaraRobot({ angles }: { angles: number[] }) {
  const [j1, j2, j3, jz] = angles;
  
  return (
    <group position={[0, -2, 0]}>
      {/* Base */}
      <Cylinder args={[0.8, 1.0, 1.5, 32]} position={[0, 0.75, 0]} material-color="#374151" />
      
      {/* Link 1 (Rotating around Y) */}
      <group position={[0, 1.5, 0]} rotation={[0, j1, 0]}>
        <Box args={[0.5, 0.4, 2.0]} position={[0, 0, 1.0]} material-color="#3b82f6" />
        
        {/* Link 2 (Rotating around Y) */}
        <group position={[0, 0.2, 2.0]} rotation={[0, j2, 0]}>
          <Cylinder args={[0.4, 0.4, 0.5, 32]} position={[0, 0, 0]} material-color="#1f2937" />
          <Box args={[0.4, 0.3, 1.8]} position={[0, 0, 0.9]} material-color="#60a5fa" />
          
          {/* Z Carriage & End Effector */}
          <group position={[0, 0, 1.8]}>
            <Cylinder args={[0.3, 0.3, 0.6, 32]} position={[0, 0, 0]} material-color="#1f2937" />
            
            {/* Z Translation */}
            <group position={[0, jz * 5, 0]} rotation={[0, j3, 0]}>
              <Cylinder args={[0.1, 0.1, 2.5, 32]} position={[0, -1.0, 0]} material-color="#d1d5db" />
              {/* Gripper Placeholder */}
              <Box args={[0.8, 0.1, 1.0]} position={[0, -2.2, 0.4]} material-color="#fbbf24" />
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}

export default function Dashboard() {
  const [isConnected, setIsConnected] = useState(false);
  const [joints, setJoints] = useState<number[]>([0, 0, 0, 0]);
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url: ROS_URL });

    rosInstance.on('connection', () => setIsConnected(true));
    rosInstance.on('error', () => setIsConnected(false));
    rosInstance.on('close', () => setIsConnected(false));

    setRos(rosInstance);

    const jointStateListener = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/joint_states',
      messageType: 'sensor_msgs/JointState'
    });

    jointStateListener.subscribe((msg: any) => {
      // Map joint states to the array (assuming standard order for scara: j1, j2, j3, jz)
      if (msg.position && msg.position.length >= 4) {
        setJoints(msg.position.slice(0, 4));
      }
    });

    return () => {
      jointStateListener.unsubscribe();
      rosInstance.close();
    };
  }, []);

  const handleJoystickMove = (e: any) => {
    if (!ros || !isConnected) return;
    console.log("Jogging: X=", e.x, " Y=", e.y);
  };

  return (
    <div className="min-h-screen bg-[#0a0a0a] text-white font-sans p-6">
      {/* Header */}
      <header className="flex justify-between items-center mb-8 border-b border-gray-800 pb-4">
        <div>
          <h1 className="text-3xl font-bold bg-gradient-to-r from-blue-400 to-indigo-500 bg-clip-text text-transparent">
            WaferFlow Control Center
          </h1>
          <p className="text-gray-400 mt-1">Remote Telemetry & 3D Visualization</p>
        </div>
        <div className="flex items-center gap-4">
          <div className={`flex items-center gap-2 px-4 py-2 rounded-full font-medium ${isConnected ? 'bg-green-900/30 text-green-400 border border-green-800' : 'bg-red-900/30 text-red-400 border border-red-800'}`}>
            <Wifi size={18} />
            {isConnected ? 'ROS 2 Connected' : 'Disconnected'}
          </div>
        </div>
      </header>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 h-[75vh]">
        
        {/* Left Column: Telemetry & Controls */}
        <div className="flex flex-col gap-6">
          {/* Telemetry Card */}
          <div className="bg-[#111111] border border-gray-800 rounded-2xl p-6 shadow-2xl">
            <div className="flex items-center gap-2 mb-4 text-blue-400">
              <Activity size={20} />
              <h2 className="text-xl font-semibold text-white">Live Joint Telemetry</h2>
            </div>
            
            <div className="space-y-4">
              {[
                { name: 'Joint 1 (Shoulder)', val: joints[0], unit: 'rad' },
                { name: 'Joint 2 (Elbow)', val: joints[1], unit: 'rad' },
                { name: 'Joint 3 (Wrist)', val: joints[2], unit: 'rad' },
                { name: 'Joint Z (Vertical)', val: joints[3], unit: 'm' },
              ].map((j, i) => (
                <div key={i} className="bg-[#1a1a1a] p-3 rounded-lg flex justify-between items-center border border-gray-800">
                  <span className="text-gray-400">{j.name}</span>
                  <span className="text-lg font-mono text-blue-300">
                    {j.val.toFixed(4)} <span className="text-xs text-gray-500">{j.unit}</span>
                  </span>
                </div>
              ))}
            </div>
          </div>

          {/* Virtual Joystick */}
          <div className="bg-[#111111] border border-gray-800 rounded-2xl p-6 shadow-2xl flex-grow flex flex-col items-center justify-center relative overflow-hidden">
            <div className="absolute top-4 left-6 flex items-center gap-2 text-indigo-400">
              <ShieldAlert size={20} />
              <h2 className="text-xl font-semibold text-white">Manual Override</h2>
            </div>
            <div className="mt-8">
              <Joystick 
                size={120} 
                sticky={false} 
                baseColor="#1f2937" 
                stickColor="#3b82f6" 
                move={handleJoystickMove} 
                stop={() => console.log("Stop Jogging")}
              />
            </div>
            <p className="text-gray-500 text-sm mt-6 text-center">
              XY Translation Override<br/>(Requires `teleop` node active)
            </p>
          </div>
        </div>

        {/* Right Column: 3D Visualization */}
        <div className="lg:col-span-2 bg-[#111111] border border-gray-800 rounded-2xl relative overflow-hidden shadow-2xl group">
          <div className="absolute top-6 left-6 z-10 flex items-center gap-2 text-indigo-400">
            <Power size={20} className="animate-pulse" />
            <h2 className="text-xl font-semibold text-white shadow-black drop-shadow-md">Digital Twin WebGL</h2>
          </div>
          
          <div className="w-full h-full bg-gradient-to-b from-[#0f172a] to-[#020617]">
            <Canvas camera={{ position: [5, 4, 5], fov: 50 }}>
              <ambientLight intensity={0.5} />
              <pointLight position={[10, 10, 10]} intensity={1.5} />
              <pointLight position={[-10, -10, -10]} intensity={0.5} color="#3b82f6" />
              
              <ScaraRobot angles={joints} />
              
              <OrbitControls makeDefault />
              <gridHelper args={[10, 10, '#334155', '#1e293b']} position={[0, -2.75, 0]} />
            </Canvas>
          </div>
          
          {/* Overlay info */}
          <div className="absolute bottom-4 right-4 bg-black/60 backdrop-blur-md px-4 py-2 rounded-lg text-xs text-gray-400 border border-white/10">
            Powered by Three.js & React Three Fiber
          </div>
        </div>

      </div>
    </div>
  );
}
