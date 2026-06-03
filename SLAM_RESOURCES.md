# SLAM Learning Resources for Beginners

Welcome to the world of Simultaneous Localization and Mapping! Since you are building a robot with a LiDAR and Encoders, you are in a perfect position to learn these concepts.

## 1. Core Concepts (The "Big Picture")
*   **[Probabilistic Robotics (The Bible)](http://www.probabilistic-robotics.org/)**: By Sebastian Thrun. Chapter 9 (Mapping) and Chapter 10 (SLAM) are the industry standards.
*   **[Cyrill Stachniss - SLAM Course (YouTube)](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_)**: This is the gold standard for university-level SLAM education. Start with "Introduction to SLAM."

## 2. Practical Algorithms for Your Hardware
Since you have a **2D LiDAR (RP-Lidar A1)** and **Differential Drive**, these are the most relevant:

### A. Hector SLAM (Great for LiDAR only)
*   **Why**: It doesn't require odometry (though it helps). It uses scan-matching at high frequency.
*   **Resource**: [Hector SLAM Paper (PDF)](https://www.sim.informatik.tu-darmstadt.de/publ/download/2011_SSRR_KohlbrecherMeyerStrykKlingauf_HectorSLAM.pdf) - Surprisingly readable for a paper.

### B. GMapping (Particle Filter SLAM)
*   **Why**: It's the most common algorithm for "dumb" robots with good odometry.
*   **Resource**: [OpenSLAM GMapping](https://openslam-org.github.io/gmapping.html).

### C. Cartographer (Modern/Advanced)
*   **Why**: Developed by Google. Uses loop closure and submaps.
*   **Resource**: [Cartographer Documentation](https://google-cartographer.github.io/cartographer/).

## 3. Implementation Steps for You
1.  **Coordinate Frames**: Understand the transform between `map` -> `odom` -> `base_link` -> `laser`.
2.  **Occupancy Grids**: Learn how to turn a (distance, angle) LiDAR point into a "pixel" in a map.
3.  **Scan Matching (ICP)**: The process of taking two laser scans and finding how much the robot moved between them.

## 4. Useful Interactive Tools
*   **[PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)**: See the SLAM section. It has Python implementations of many algorithms that are easy to step through with a debugger.
*   **[TinySLAM](https://github.com/Slamer/tinySLAM)**: A very minimal C implementation of SLAM that is great for understanding the bare minimum code required.

---
**Gemini Tip**: Start with **"Scan Matching."** If you can align two consecutive LiDAR scans to find the displacement, you have already solved 50% of SLAM!
