// From https://ksp-kos.github.io/KOS/tutorials/exenode.html
//
// modified for spin-stabilized stage with control avionics on exhausted
// prior stage that is decoupled at burn initiation, and unthrottle-able
// engines with ullage requirements.
//
// Invocation, with spin-up time 30, then burn with lead time 20 seconds.
//
//     RUNPATH("0:/execute_spin_stabilized_insertion_burn.ks",60,20).
//
// Be careful not to run out of RCS fuel during spin-up or post-spin-up coasting,
// or it might affect tank ullage and post-spin orientiation.
//
// Will tend to overshoot slightly, as it doesn't throttle down. So make your
// nodes a bit low to compensate.
//
// TODO: support taking an orbital period target instead of a dV target,
//       and adjust the next node, or take a time to burn and make a node.
//
// TODO: smarter ullage, only fire when needed, and only for a short time.
// TODO: smarter spin-up, target RPM or angular velocity.
// TODO: print an estimate of when to warp to, or support auto-warp
// TODO: use RCS to retro burn if available even on science core?
// TODO: multiple stages
// TODO: estimate the dV jump between iterations and prepare an early cut-off
//       if the next iteration will exceed target burn? Tends to overshoot too much.
//       or provide a margin and switch to rcs?
// TODO: target orbital period
// TODO: engine shutdown latency?
// TODO: stop it trying to spin down during point stabilization
//
// TODO: discover RCS on final stage, and if found, cut engine early and
// use the RCS for the final refinement burn.:
//
// node 2428.4 in testing


// set new_node_time to timespan(0,0,0,4,0). set new_node_dv to 2427.5. set nd to NODE(new_node_time, 0, 0, new_node_dv). add nd.

parameter spin_up_time is 0. // seconds
parameter burn_lead_time is 0. // seconds

// TODO spin rate goal. Would have to estimate ship's angular mass or something
// to estimate the spin up time and thus lead time.
//parameter target_spin_rate is 1. // rad/s; 1 radian per second ~= 9.5493 RPM

// for testing
set new_node_time to timespan(0,0,0,4,0).
set new_node_dv to 2427.5.

// Cut main engines and use RCS for the final burn after this dV.
// TODO also add for orbital period margin
// TODO automatic if RCS detected
// TODO estimate RCS stage available dV
set rcs_for_final_ms to 10.

// Orbital period target for the burn.
// TODO: make this a parameter or a margin around the node
set targetPeriodMin to timespan(0,0,11,56,0):seconds.
set targetPeriodMax to timespan(0,0,12,0,2):seconds.

// Time to refine the orientation after spinning up the stage.
// TODO define as angular momentum instead?
set spin_stablize_margin to 10. // seconds

// hack for automation of the test
//print "autotest hack".
//set initial_stage to 2.
//until ship:stagenum = initial_stage {
//        print "staging to get to test stage.".
//        set laststage to ship:stagenum.
//        stage.
//        wait 0.5.
//}
//lock throttle to 1.
//wait 1.
//lock throttle to 0.

if hasnode
{
    print "using existing node".
    set nd to nextnode.
}
else
{
    print "creating new node in " + new_node_time:full + " at " + new_node_dv + "m/s".
    set nd to NODE(new_node_time, 0, 0, new_node_dv).
    add nd.
}

//print out node's basic parameters - ETA and deltaV
print "Node in: " + round(nd:eta) + ", DeltaV: " + round(nd:burnvector:mag).

// Detect RCS on current and next stage.
//
// TODO detect where the RCS is attached, not where it's staged, since it could
// be enabled on an earlier stage in the sequence and still be available for
// use.
//
set rcs_on_boost_stage to false.
set rcs_on_payload_stage to false.
for thruster in ship:rcs {
        if thruster:availablethrust > 0 and thruster:stage = ship:stagenum - 1 {
                set rcs_on_boost_stage to true.
        }
        if thruster:availablethrust > 0 and thruster:stage = ship:stagenum - 2 {
                set rcs_on_payload_stage to true.
        }
}
print "RCS detection: boost stage " + (ship:stagenum - 1) + " has " + rcs_on_boost_stage + ", next stage " + (ship:stagenum - 2) + " has " + rcs_on_payload_stage + " RCS.".
if rcs_for_final_ms > 0 and (rcs_on_boost_stage or rcs_on_payload_stage) {
        // TODO compute the dV available on the RCS stage, estimate the error
        // from the main engines by tick vs acceleration at near target dV, and
        // use that to compute how early to cut the engines and use RCS.
        print "RCS detected on current stage, will use for final " + rcs_for_final_ms + "m/s burn.".
} else {
        print "No RCS detected, will burn to target dV ignoring rcs burn request".
        set rcs_for_final_ms to 0.
}

SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.

print "Pointing at burn vector.".
rcs on.
set np to nd:burnvector. //points to node, don't care about the roll direction.
lock steering to np.

//now we need to wait until the burn vector and ship's facing are aligned
print "Waiting for stable alignment...".
// FIXME: should ignore the y component since we don't much care if we're spinning already,
// and steering may not compoensate
wait until vang(np, ship:facing:vector) < 0.05 and ship:angularmomentum:mag < 0.01.

// The ship is facing the right direction, let's wait for spin-up time.
// We'll decouple the guided avionics on burn initiation so the spin has
// to come first.
// TODO: wait for angular velocity to be stable, not just facing.
//
print "Aligned: " + vang(np, ship:facing:vector) + ", angularmomentum " + ship:ANGULARMOMENTUM + " (magnitude " + ship:angularmomentum:mag + "), angularvel " + ship:ANGULARVEL + " (magnitude " + ship:ANGULARVEL:mag + ").".

print "Warping to spin-up.".
kuniverse:timewarp:warpto(nd:time - (spin_up_time + spin_stablize_margin + burn_lead_time - 20)).
wait until nd:eta <= (spin_up_time + spin_stablize_margin + burn_lead_time).

print "Starting ullage thrust.".
set ship:control:fore to 1. 

// ship:angularmomentum:y is the angular momentum around the y axis. The
// ship will be spinning around the y axis, but we want zero on the x and z
// components.
//wait until ship:angularmomentum:y.

// Spin up the stage. This might cause loss of directional control; we'll
// re-stabilize once spun up; a 5 second built-in margin is provided for this.
// Should really set a target RPM or angular velocity, but this is a simple'
// example.
print "Spinning up stage.".
//unlock steering. // see what happens
lock steering to np.
set ship:control:roll to 1.
wait spin_up_time.
//set ship:control:roll to 0. // see what happens
print "Spin-up complete.".

// Re-point the stage at the node while preparing for burn
// and settling propellent.
print "Refining orientation and waiting for burn initiation.".
lock steering to np.
wait until nd:eta <= burn_lead_time.

// initial deltav for burn. Used to determine burn vector,
// not for burn timing.
set dv0 to np:vec.

// Initiate the un-throttle-able burn, decoupling the prior stage and
// losing guidance. Disable ullage RCS in case it can still fire and might
// interfere with the burn.
set burnStartTime to time().
print "Initiating burn at " + burnStartTime:CLOCK.
lock throttle to 1.
set ship:control:fore to 0. 
stage.

print "Heading error at burn initiation: " + vang(np, ship:facing:vector).

set done to False.
set using_rcs to False.
until done
{
    // No point looping faster than the next physics tick.
    wait 0.

    set dvTime to time() - burnStartTime.
    set period to ship:orbit:period.

    if period >= targetPeriodMin and period <= targetPeriodMax
    {
        lock throttle to 0.
        print "Target period reached: " + timespan(period):full.
        break.
    }

    // This (round(nd:deltav:mag,1) < rcs_for_final_ms
    // doesn't work because it counts the directional error in the burn vector.
    // We need only the prograde magnitude of the vector. Maybe this can be done
    // with a vector projection?
    // or by using the orbital period instead of the dv as the threshold.
    //   print VDOT(ship:facing:forevector, nd:burnvector).
    if (not using_rcs) and (rcs_for_final_ms > 0) {
        set remaining_prograde_burn to VDOT(ship:facing:forevector, nd:burnvector).
        if (remaining_prograde_burn <= rcs_for_final_ms)
        {
            // Cut engines immediately. Using throttle first should help it be
            // symmetric, then explicitly shut them down so we can use RCS
            // throttle.
            lock throttle to 0.
            for engine in ship:engines {
                    if engine:ignition {
                            engine:shutdown().
                    }
            }
            print "Engines shut down early to use RCS for configured final burn of up to " + rcs_for_final_ms + " m/s.".
            print "Remaining prograde component: " + remaining_prograde_burn + " m/s.".
            print "Node total dV error: " + round(nd:deltav:mag,1) + " m/s.".
            print "Period: " + timespan(period):full.
            print "Heading error from original vector at engine shutdown: " + vang(dv0, ship:facing:vector).
            if not rcs_on_boost_stage {
                    print "No RCS on engine stage, decoupling current stage.".
                    stage.
            }
            print "Enabling RCS for final burn.".
            for thruster in ship:rcs {
                    set thruster:enabled to true.
                    set thruster:forebythrottle to true.
            }
            lock throttle to 1.
            set using_rcs to true.
        }
    }

    // Cut the throttle as soon as our nd:deltav and initial deltav start
    // facing opposite directions this check is done via checking the dot
    // product of those 2 vectors
    set dvNow to vdot(dv0, nd:deltav).
    if dvNow < 0
    {
        // We've met or overshot our dV target
        lock throttle to 0.
        print "Target dV reached: " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(dvNow,1).
        break.
    }

    // This print is slow, and might make us miss our target, so only uncomment for debugging.
    //print "burning, dvNow: " + round(dvNow,1) + " at " + dvTime + "s with period " + timespan(period):full.
}

unlock steering.
unlock throttle.

print "Burn complete, inserted with period " + timespan(ship:orbit:period):full + ", prograde dV error " + remaining_prograde_burn + " m/s and total dV error " + round(nd:deltav:mag,1) + " m/s.".

print "Heading error at burn termination: " + vang(dv0, ship:facing:vector).

wait 1.

//we no longer need the maneuver node
//remove nd.

//set throttle to 0 just in case.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.

// vim: et sw=4 ts=4 ai
