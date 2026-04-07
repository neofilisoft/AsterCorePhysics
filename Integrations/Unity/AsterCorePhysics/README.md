# AsterCore Physics for Unity

This package integrates AsterCore as a middleware physics runtime for Unity. It does not replace Unity Physics globally. Instead, it gives you an `AsterCoreWorldBehaviour` plus `AsterCoreBodyBehaviour` components that step a native AsterCore world and sync GameObject transforms.

## Install

1. Add this folder as a local package in Unity Package Manager.
2. Build `AsterCoreCAPI.dll` from the AsterCore repo.
3. Copy `AsterCoreCAPI.dll` into `Runtime/Plugins/Windows/x86_64/`.
4. Add `AsterCoreWorldBehaviour` to a scene object.
5. Add `AsterCoreBodyBehaviour` to any GameObject you want AsterCore to control.
6. Remove or disable Unity `Rigidbody` / `Collider` components on those GameObjects to avoid conflicting simulation.

## What Works

- world creation and stepping
- gravity control
- box and sphere body creation
- transform sync for dynamic and static bodies
- linear and angular velocity sync
- raycast queries

## Scope

This package is intended as a middleware bridge. It is a practical way to integrate AsterCore into Unity gameplay code today without pretending to replace every built-in Unity physics workflow.
