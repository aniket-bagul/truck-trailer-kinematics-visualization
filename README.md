# 🚛 Visualisation of Kinematics of a Truck with Trailer

## 📌 Project Overview

This project focuses on the modelling, simulation, and 3D visualisation of the kinematic behaviour of a truck with a trailer.

The main objective is to study how the truck and trailer move together during different driving manoeuvres, particularly during turning. The vehicle motion is calculated in MATLAB and visualised using a 3D VRML model.

## 🎯 Objectives

* Develop a kinematic model of a truck-trailer system
* Simulate the vehicle motion using MATLAB
* Visualise the truck and trailer movement in a 3D environment
* Visualise the trajectory and articulation of the trailer

## 🛠️ Technologies & Tools

* MATLAB
* VRML
* Blaxxun Contact / VRML Viewer
* Kinematic Modelling
* 3D Visualisation
* Numerical Simulation

## 📁 Project Structure

```text
truck-trailer-kinematics-visualization/
│
├── Images/
│   └── Simulation screenshots
│
├── MATLAB/
│   └── MATLAB simulation files
│
├── Textures/
│   └── Wheel and model textures
│
├── VRML/
│   └── Truck-trailer 3D model
│
├── README.md
└── .gitignore
```

## 🔬 Methodology

The truck-trailer system is represented using a kinematic model that describes the position and orientation of the truck and trailer.

MATLAB is used to calculate the vehicle motion based on parameters such as vehicle velocity, steering angle, wheelbase, and trailer geometry.

The calculated position and orientation are then used to control the corresponding 3D truck-trailer model in the VRML visualisation environment.

This provides a visual representation of how the trailer follows the truck during different manoeuvres.

## 📐 Kinematic Model

The model considers important parameters including:

* Truck position `(x, y)`
* Truck orientation
* Steering angle
* Vehicle velocity
* Trailer orientation
* Distance between truck and trailer axles
* Articulation angle

The trailer does not follow exactly the same path as the truck. Its position and orientation depend on the motion and orientation of the truck, resulting in an offset trajectory, particularly during turning.

## 🔄 MATLAB–VRML Visualisation

The simulation is divided into two main parts:

**1. MATLAB Simulation**

MATLAB calculates the kinematic behaviour and determines the position and orientation of the truck and trailer.

**2. VRML Visualisation**

The calculated motion is represented using a 3D VRML truck-trailer model. This allows the vehicle movement and trailer articulation to be observed visually.

```text
Vehicle Parameters
        ↓
MATLAB Kinematic Model
        ↓
Position & Orientation
        ↓
VRML 3D Model
        ↓
Truck-Trailer Visualisation
```

## 🎥 Visualisation

The 3D visualisation allows the following behaviours to be observed:

* Truck movement
* Trailer movement
* Steering behaviour
* Trailer articulation
* Turning trajectory
* Relative motion between truck and trailer

## 📊 Results

The simulation demonstrates the kinematic behaviour of the truck-trailer system during vehicle manoeuvres.

The 3D visualisation clearly shows how the trailer responds to the truck's movement and how its trajectory differs from the truck's trajectory during turning.

## 🚀 Future Improvements

Possible extensions of this project include:

* Integration of GPS, IMU, and wheel encoder data
* Real-time sensor integration
* Autonomous path planning
* Obstacle avoidance
* Multiple-trailer configurations
* Real-time 3D visualisation using WebGL
* Integration with ROS2
* Autonomous truck-trailer navigation

## 👨‍💻 Author

**Aniket Kailas Bagul**

M.Sc. Mechatronics
Focus: Robotics & Autonomous Systems

---

⭐ This scientific project was developed as part of my mechatronics studies to explore vehicle kinematics, simulation, and 3D visualisation.
