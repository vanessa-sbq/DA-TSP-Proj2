
# Travelling Salesperson Problem (TSP) Project

## Overview

Welcome to the Travelling Salesperson Problem (TSP) Project! This project, developed by three students at the University of Porto, FEUP, aims to analyze and design heuristics to solve the TSP using datasets from ocean shipping and urban deliveries. The goal is to develop both exact and approximate solutions to understand the trade-offs between optimality and efficiency in solving this intractable problem.

## Objectives

The main objectives of this project are:
1. Implement a basic exhaustive approach for the TSP to understand its computational limits.
2. Develop and analyze a set of heuristics to provide approximate solutions to the TSP.
3. Enhance critical thinking skills through the development and comparison of different algorithmic approaches.

## Dataset

The project utilizes multiple datasets that describe points of delivery in urban settings and harbors in the context of ocean shipping. These datasets include:
- Small toy examples for validating exact solutions.
- Medium-sized graphs for optimizing heuristic algorithms.
- Scaled-down versions of real-world graphs for testing algorithm performance on realistic scenarios.

## Features

### Data Handling
- Loaded and parsed provided datasets to create appropriate graph structures for TSP analysis.

### User Interface
- Developed a user-friendly interface to expose all implemented functionalities and facilitate demonstration.

## Functionalities

1. **Backtracking Algorithm**
   - Implemented a backtracking approach to find the optimal TSP solution for small graphs.
   - Validated the approach using small toy graphs and illustrate the feasibility limits for larger graphs.

2. **Triangular Approximation Heuristic**
   - Implemented a 2-approximation algorithm leveraging the triangular inequality.
   - Compared the results with the backtracking approach for small graphs and apply the heuristic to larger graphs using the Haversine distance for missing edges.

3. **Additional Heuristics**
   - Designed and implemented an efficient heuristic of choice to solve the TSP.
   - Focused on achieving a balance between solution quality and execution time.
   - Compared the heuristic with the triangular approximation algorithm in terms of performance and optimality.

4. **TSP in the Real World**
   - Developed an algorithm to provide TSP solutions for real-world, non-fully connected graphs.
   - Ensured the algorithm can handle arbitrary starting points and provide near-optimal solutions in feasible time.
   - Discussed the complexity and feasibility of TSP solutions in real-world scenarios.

## Documentation

- Included Doxygen documentation for all implemented code, indicating the time complexity of each algorithm.

## Implementation Details

This project is implemented in C++ and involves the development of various algorithms to address the TSP. The project emphasizes understanding the computational challenges and devising effective heuristic solutions for realistic applications.

## Authors

Leonardo Garcia<br/>
Antonio Santos<br/>
Vanessa Queirós<br/>

### Thank you!

