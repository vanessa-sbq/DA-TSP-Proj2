
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
- Load and parse provided datasets to create appropriate graph structures for TSP analysis.

### User Interface
- Develop a user-friendly interface to expose all implemented functionalities and facilitate demonstration.

## Functionalities

1. **Backtracking Algorithm**
   - Implement a backtracking approach to find the optimal TSP solution for small graphs.
   - Validate the approach using small toy graphs and illustrate the feasibility limits for larger graphs.

2. **Triangular Approximation Heuristic**
   - Implement a 2-approximation algorithm leveraging the triangular inequality.
   - Compare the results with the backtracking approach for small graphs and apply the heuristic to larger graphs using the Haversine distance for missing edges.

3. **Additional Heuristics**
   - Design and implement an efficient heuristic of choice to solve the TSP.
   - Focus on achieving a balance between solution quality and execution time.
   - Compare the heuristic with the triangular approximation algorithm in terms of performance and optimality.

4. **TSP in the Real World**
   - Develop an algorithm to provide TSP solutions for real-world, non-fully connected graphs.
   - Ensure the algorithm can handle arbitrary starting points and provide near-optimal solutions in feasible time.
   - Discuss the complexity and feasibility of TSP solutions in real-world scenarios.

## Documentation

- Include Doxygen documentation for all implemented code, indicating the time complexity of each algorithm.

## Implementation Details

This project is implemented in C++ and involves the development of various algorithms to address the TSP. The project emphasizes understanding the computational challenges and devising effective heuristic solutions for realistic applications.

## Demonstration & Presentation

Prepare a concise 15-minute presentation highlighting:
- The implemented TSP solutions and their analysis.
- The trade-offs between efficiency and optimality for different algorithms.
- The performance comparison of heuristics and backtracking approaches using provided datasets.

## Authors

Leonardo Garcia<br/>
Antonio Santos<br/>
Vanessa Queirós<br/>

### Thank you!

