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
The repository structure is intentionally minimal at this stage and will evolve as
artefacts are added during the coursework.

---

## Branch Naming Convention

Branches are named to reflect **intent and activity**.  
Requirement identifiers shall only be included when a branch directly targets those
requirements.

---

### 1. Branches Targeting a Single Requirement
Use this pattern when a branch directly implements, verifies, or analyses **one**
specific requirement.

req/<REQUIREMENT-ID>-<short-description>

makefile
Copy code

Example:
req/LGCS-FR001-deploy-function

yaml
Copy code

---

### 2. Branches Targeting Multiple Requirements
Use this pattern when a single change directly affects **multiple related requirements**.

req/<REQUIREMENT-ID-START>-<REQUIREMENT-ID-END>-<short-description>

makefile
Copy code

Example:
req/LGCS-SR001-002-altitude-sensor

yaml
Copy code

Only group requirements when the relationship is clear and defensible.

---

### 3. Branches Not Targeting Specific Requirements
When no direct requirement traceability exists, use a **task-based naming pattern**.

task/<short-description>

makefile
Copy code

Example:
task/add_script_header_template

yaml
Copy code

This pattern shall be used for:
- Tooling
- Documentation
- Refactoring
- Templates
- General repository improvements

---

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

python
Copy code
Targeted Requirements:
- None (supporting analysis or tooling only)
