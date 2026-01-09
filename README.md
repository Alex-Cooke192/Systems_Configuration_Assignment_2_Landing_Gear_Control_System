# Landing Gear Control System (LGCS) – Coursework Repository

## Overview
This repository contains Python scripts and supporting artefacts developed as part of
the LGCS coursework assignment. The material supports requirements analysis, performance
evaluation, fault-handling studies, and presentation artefacts related to the Landing Gear
Control System.

All code in this repository is **academic and non-operational**. It is intended solely
for analysis, illustration, and assessment purposes.

---

## Repository Structure

## Branch Naming Convention

Branches are named to reflect **intent and activity**, not assumed requirement
implementation, unless explicitly stated.



### Default Pattern (Recommended)
task/<short-description>

markdown
Copy code

Examples:
- `task/performance-analysis`
- `task/requirements-cleanup`
- `task/plot-generation`

### Other Accepted Patterns
Use when appropriate:

analysis/<topic> # Analytical or study work
docs/<topic> # Documentation or formatting changes
test/<topic> # Test or verification scripts
refactor/<topic> # Structural or code quality changes
experiment/<topic> # Exploratory or non-final work
spike/<topic> # Short-lived investigation

markdown
Copy code

### Avoid
Do **not** name branches after requirements unless they directly implement or verify them:
lgcs-fr001
feature/gear-deploy

yaml
Copy code

---

## Python Script Header Template

Each Python script shall include a **block comment header** describing its purpose,
scope, and traceability. The following template must be placed at the top of every
`.py` file in this repository.

```python
"""
Title: <Short descriptive title>
Author: <Your Name>
Date Created: <YYYY-MM-DD>
Last Modified: <YYYY-MM-DD>
Version: <e.g. 1.0>

Purpose:
Brief description of what the script does and why it exists.

Targeted Requirements:
- <Requirement ID>: <Short description>
- <Requirement ID>: <Short description>
(Use "None" if no direct requirement traceability exists.)

Scope and Limitations:
- Assumptions made in the model or analysis
- Known limitations or simplifications

Safety Notice:
This software is for academic and illustrative purposes only.
It is not flight-certified and must not be used in operational systems.

Dependencies:
- Python <version>
- External libraries (if any)

Related Documents:
- LGCS Requirements Specification
- Relevant appendix or system document
"""
If a script does not target a specific requirement, explicitly state:

sql
Copy code
Targeted Requirements:
- None (supporting analysis / tooling only)