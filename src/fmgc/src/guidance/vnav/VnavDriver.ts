/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Working Title, FlyByWire Simulations
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Auto-added?
import { WayPoint } from '@fmgc/types/fstypes/FSTypes';

import { LateralMode, VerticalMode } from '@shared/autopilot';
import { ManagedFlightPlan } from '@fmgc/flightplanning/ManagedFlightPlan';
import { FlightPlanManager } from '@fmgc/flightplanning/FlightPlanManager';
import { SegmentType } from '@fmgc/flightplanning/FlightPlanSegment';
import { GuidanceComponent } from '../GuidanceComponent';
import { Leg, TFLeg } from '../Geometry';
import { GuidanceController } from '../GuidanceController';

export class VnavDriver implements GuidanceComponent {
    private guidanceController: GuidanceController;

    public fpm: FlightPlanManager;

    public activeWaypoint: WayPoint;

    public verticalFlightPlan: VerticalWaypoint[];

    public verticalFlightPlanSegments: VerticalSegment[];

    public firstPossibleDescentWaypointIndex: number;

    public lastClimbWaypointIndex: number;

    public firstPathSegmentIndex: number;

    public pathExists: boolean;

    public activeConstraint: Constraint;

    public atConstraints: AtConstraint[];

    public firstDescentConstraintIndex: number;

    private fpChecksum: number;

    constructor(guidanceController: GuidanceController, fpm: FlightPlanManager) {
        this.guidanceController = guidanceController;
        this.fpm = fpm;

        this.activeWaypoint = undefined;
        this.verticalFlightPlan = [];
        this.verticalFlightPlanSegments = [];

        this.firstPossibleDescentWaypointIndex = 0;
        this.lastClimbWaypointIndex = 0;

        // The index of the first path segment (path segments run in reverse order to 0,
        // so the first one is the highest number).
        this.firstPathSegmentIndex = 0;

        // Whether or not a descent path has been calculated/exists
        this.pathExists = false;

        // The next constraint
        this.activeConstraint = undefined;

        // The at constraints in the current plan for segment building.
        this.atConstraints = [];

        // The flight plan index of the waypoint with the first descent constraint
        this.firstDescentConstraintIndex = undefined;

        // The checksum to compare against the flight plan.
        this.fpChecksum = -1;
    }

    /**
     * The active flight plan.
     * @type {ManagedFlightPlan}
     */
    get flightplan(): ManagedFlightPlan {
        return this.fpm.getFlightPlan(0);
    }

    get currentWaypoints(): WayPoint[] {
        return this.flightplan.waypoints.slice(this.flightplan.activeWaypointIndex);
    }

    get allWaypoints(): WayPoint[] {
        return this.flightplan.waypoints;
    }


    init(): void {
        console.log('[FMGC/Guidance] VnavDriver initialized!');
    }

    update(_deltaTime: number): void {
        // TODO: Write code for following operations:

        // Update active lateral waypoint
        // Get vertical segment from active waypoint

        // Check if conditions for VNAV to update/run are valid
        // If VNAV can be run:
        // - Update current position and distance from active waypoint
        // - If flight plan is changed:
        //      - Re-build vertical flight plan
        // - Manage constraints
        // - Calculate ToD
        //      - ToD present if we haven't descended to first
    }

    buildVerticalFlightPlan(): void {
        // TODO: IDLE SEGMENT

        const waypointCount = this.allWaypoints.length;

        this.verticalFlightPlan = [];
        this.atConstraints = [];
        this.activeConstraint = undefined;
        this.firstDescentConstraintIndex = undefined;

        let lastClimbWaypointIndex = 0;
        let firstPossibleDescentWaypointIndex = 0;
        let firstApproachWaypointIndex = this.getFirstApproachWaypointIndex();
        let lastApproachWaypointIndex = this.getLastApproachWaypointIndex();

        for (let i = 0; i < waypointCount; i++) {
            const segmentType = this.fpm.getSegmentFromWaypoint(this.allWaypoints[i]).type;
            const isClimb = !!((segmentType === SegmentType.Departure || segmentType === SegmentType.Missed));
            const constraints = this.parseConstraints(this.allWaypoints[i]);
            const VWP = new VerticalWaypoint(i, this.allWaypoints[i].ident, isClimb);

            VWP.legDistanceTo = i > 0 ? this.allWaypoints[i].cumulativeDistanceInFP - this.allWaypoints[i - 1].cumulativeDistanceInFP : 0;
            VWP.upperConstraintAltitude = constraints.upperConstraint;
            VWP.lowerConstraintAltitude = constraints.lowerConstraint;
            VWP.isAtConstraint = constraints.isAtConstraint;
            VWP.hasConstraint = constraints.hasConstraint;

            // Check if current waypoint is the first descent constraint waypoint
            // If it is, then set the appropriate variable's index to current index
            if (this.firstDescentConstraintIndex === undefined && !isClimb && constraints.hasConstraint) {
                this.firstDescentConstraintIndex = i;
            }

            // If current waypoint is part of approach, turn into "AT" constraint
            if (firstApproachWaypointIndex !== undefined && i >= firstApproachWaypointIndex && VWP.lowerConstraintAltitude > 0) {
                VWP.upperConstraintAltitude = constraints.lowerConstraint;
                VWP.isAtConstraint = true;
                firstApproachWaypointIndex = undefined;
                // console.log("setting " + vwp.ident + " as first approach waypoint AT constraint " + constraints.lowerConstraint + "FT");
            }

            // Assign target altitude if "AT" constraint
            if (VWP.isAtConstraint || (VWP.hasConstraint && VWP.upperConstraintAltitude < Infinity)) {
                if (VWP.isAtConstraint) {
                    VWP.waypointFPTA = VWP.upperConstraintAltitude;
                }
                const atConstraint: AtConstraint = {
                    index: i,
                    altitude: VWP.upperConstraintAltitude,
                };
                // console.log("at constraint " + atConstraint.index + " " + vwp.ident);
                this.atConstraints.push(atConstraint);
            }

            // Add vertical waypoint to vertical flight plan and update some index variables
            this.verticalFlightPlan.push(VWP);
            lastClimbWaypointIndex = (isClimb && i < lastApproachWaypointIndex) ? i : lastClimbWaypointIndex;
            firstPossibleDescentWaypointIndex = (isClimb && VWP.hasConstraint && i < lastApproachWaypointIndex) ? i : firstPossibleDescentWaypointIndex;

            const segmentBuildStartIndex = Math.max(this.flightplan.activeWaypointIndex - 1, lastClimbWaypointIndex);

            // TODO: Build vertical segments
            this.verticalFlightPlanSegments = [];
            const nextSegmentEndIndex = undefined;
            let segmentIndex = 0;
            for (let j = lastApproachWaypointIndex; j > segmentBuildStartIndex; j--) {
                // console.log("j:" + j + ", " + this.verticalFlightPlan[j].ident + " segment:" + this.verticalFlightPlan[j].segment + " FPTA:" + this.verticalFlightPlan[j].waypointFPTA);
                if (!this.verticalFlightPlan[j].segment && this.verticalFlightPlan[j].waypointFPTA) {
                    this.verticalFlightPlanSegments.push(this.buildVerticalSegment(segmentIndex, j));
                    segmentIndex++;
                }
            }

            this.lastClimbWaypointIndex = lastClimbWaypointIndex;
            this.firstPossibleDescentWaypointIndex = firstPossibleDescentWaypointIndex;
            this.firstPathSegmentIndex = this.verticalFlightPlanSegments.length - 1;
            const isPath = !(segmentIndex === 0 && nextSegmentEndIndex === undefined);

            // TODO: VNAV STATE
        }
    }

    buildVerticalSegment(segmentIndex: number, j: number): VerticalSegment {
        // pass
    }

    parseConstraints(waypoint: WayPoint): Constraint {
        const constraints = {
            upperConstraint: Infinity,
            lowerConstraint: 0,
            isAtConstraint: false,
            hasConstraint: false,
        };
        switch (waypoint.legAltitudeDescription) {
        case 1:
            // AT constraint
            constraints.upperConstraint = Math.floor(waypoint.legAltitude1);
            constraints.lowerConstraint = Math.floor(waypoint.legAltitude1);
            constraints.isAtConstraint = true;
            constraints.hasConstraint = true;
            break;
        case 2:
            // AT OR ABOVE constraint
            constraints.lowerConstraint = Math.floor(waypoint.legAltitude1);
            constraints.hasConstraint = true;
            break;
        case 3:
            // AT OR BELOW constraint
            constraints.upperConstraint = Math.floor(waypoint.legAltitude1);
            constraints.hasConstraint = true;
            break;
        case 4:
            // Altitude range constraint
            constraints.lowerConstraint = Math.floor(waypoint.legAltitude2);
            constraints.upperConstraint = Math.floor(waypoint.legAltitude1);
            constraints.hasConstraint = true;
            break;
        case 5:
            // AT OR ABOVE constraint, using alt2
            constraints.lowerConstraint = Math.floor(waypoint.legAltitude2);
            constraints.hasConstraint = true;
            break;
        default:
            break;
        }
        return constraints;
    }

    getFirstApproachWaypointIndex(): number {
        const approach = this.fpm.getApproachWaypoints();
        if (approach && approach.length > 0) {
            return this.allWaypoints.indexOf(approach[0]);
        }
        return undefined;
    }

    getLastApproachWaypointIndex(): number {
        const approach = this.fpm.getApproachWaypoints();
        if (approach && approach.length > 0) {
            return this.allWaypoints.indexOf(approach[approach.length - 1]);
        }
        return undefined;
    }
}

class VerticalWaypoint {
    public indexInFlightPlan: number;

    public ident: string;

    public waypointFPA: number;

    public waypointFPTA: number;

    public upperConstraintAltitude: number;

    public lowerConstraintAltitude: number;

    public upperConstraintFPA: number;

    public lowerConstraintFPA: number;

    public legDistanceTo: number;

    public isClimb: boolean;

    public isAtConstraint: boolean;

    public hasConstraint: boolean;

    public segment: VerticalSegment;

    constructor(index = undefined, ident = undefined, isClimb = false) {
        // The waypoint's index in the lateral flight plan
        this.indexInFlightPlan = index;

        // The ident of the vertical waypoint.
        this.ident = ident;

        // The calculated flight path angle TO the waypoint.
        this.waypointFPA = undefined;

        // The calculated flight plan target altitude for the waypoint.
        this.waypointFPTA = undefined;

        // The highest altitude allowed at this vertical wapyoint.
        this.upperConstraintAltitude = undefined;

        // The lowest altitude allowed at this vertical wapyoint.
        this.lowerConstraintAltitude = undefined;

        // The FPA from the upper constraint altitude to the next fixed vnav target.
        this.upperConstraintFPA = undefined;

        // The FPA from the lower constraint altitude to the next fixed vnav target.
        this.lowerConstraintFPA = undefined;

        // The leg distance from the prior waypoint to this waypoint.
        this.legDistanceTo = undefined;

        // Whether this waypoint is part of the climb or not.
        this.isClimb = isClimb;

        // Whether this waypoint is an AT constraint.
        this.isAtConstraint = false;

        // Whether this waypoint has a constraint.
        this.hasConstraint = false;

        // Which vertical path segment is this waypoint part of.
        this.segment = undefined;
    }
}

class VerticalSegment {
    // TODO: Add support for IDLE and repressurization segments

    public startIndex: number;

    public targetIndex: number;

    public fpa: number;

    public distanceToNextTod: number;

    public segmentStartsLevel: boolean;

    public segmentEndsLevel: boolean;

    constructor(startIndex = undefined, targetIndex = undefined, fpa = undefined, distanceToNextTod = 0, segmentStartsLevel = false, segmentEndsLevel = false) {
        // The first waypoint index of this segment.
        this.startIndex = startIndex;

        // The last waypoint index of this segment and the vertical target of this segment.
        this.targetIndex = targetIndex;

        // The segment flight path angle (fpa).
        this.fpa = fpa;

        // The distance from the end of this segment to the next TOD;
        // 0 if it is a continuous descent or at the end of the path.
        this.distanceToNextTod = distanceToNextTod;

        // If the segment starts flat/with a TOD.
        this.segmentStartsLevel = segmentStartsLevel;

        // If the segment ends flat/level.
        this.segmentEndsLevel = segmentEndsLevel;
    }
}

class EngineModel {
    /**
     * Table 1502 - CN2 vs CN1 @ Mach 0, 0.2, 0.9
     * n2_to_n1_table
     * @param i row index (n2)
     * @param j 1 = Mach 0, 2 = Mach 0.2, 3 = Mach 0.9 
     * @returns Corrected N1 (CN1)
     */
    static table1502(i: number, j: number): number {
        const t = [ 
            [18.200000, 0.000000, 0.000000, 17.000000],
            [22.000000, 1.900000, 1.900000, 17.400000],
            [26.000000, 2.500000, 2.500000, 18.200000],
            [57.000000, 12.800000, 12.800000, 27.000000],
            [68.200000, 19.600000, 19.600000, 34.827774],
            [77.000000, 26.000000, 26.000000, 40.839552],
            [83.000000, 31.420240, 31.420240, 44.768766],
            [89.000000, 40.972041, 40.972041, 50.092140],
            [92.800000, 51.000000, 51.000000, 55.042000],
            [97.000000, 65.000000, 65.000000, 65.000000],
            [100.000000, 77.000000, 77.000000, 77.000000],
            [104.000000, 85.000000, 85.000000, 85.500000],
            [116.500000, 101.000000, 101.000000, 101.000000],
        ];

        return t[i][j];
    }

    /**
     * Table 1503 - Turbine LoMach (0) CN2 vs. Throttle @ IAP Ratio 1.00000000, 1.20172257, 1.453783983, 2.175007333, 3.364755652, 4.47246108, 5.415178313
     * mach_0_corrected_commanded_ne_table
     * @param i row index (thrust lever position)
     * @param j IAP ratio
     * @returns Corrected N2 (CN2)
     */
    static table1503(i: number, j: number): number {
        const t = [
            [0.000000, 68.200000, 69.402657, 70.671269, 73.432244, 76.544349, 78.644882, 78.644882],
            [0.100000, 76.000000, 77.340205, 78.753906, 81.830654, 85.298688, 87.639458, 87.639458],
            [0.200000, 83.000000, 84.463645, 86.007556, 89.367688, 93.155146, 95.711513, 95.711513],
            [0.400000, 92.800000, 94.436461, 96.162664, 99.919535, 104.154188, 107.012390, 107.012390],
            [0.600000, 98.000000, 99.728159, 101.551090, 105.518475, 109.990414, 113.008774, 113.008774],
            [0.750000, 101.500000, 103.289879, 105.177914, 109.286991, 113.918643, 117.044802, 117.044802],
            [0.900000, 103.000000, 104.816330, 106.000000, 110.902070, 115.602170, 118.774528, 118.774528],
            [1.000000, 104.200000, 106.037491, 107.975750, 112.194133, 116.948991, 120.158309, 120.158309],
        ];

        return t[i][j];
    }

    /**
     * Table 1504 - Turbine HiMach (0.9) CN2 vs. Throttle @ IAP Ratio 1.00000000, 1.20172257, 1.453783983, 2.175007333, 3.364755652, 4.47246108, 5.415178313
     * mach_hi_corrected_commanded_ne_table
     * @param i row index (thrust lever position)
     * @param j IAP ratio
     * @returns Corrected N2 (CN2)
     */
    static table1504(i: number, j: number): number {
        const t = [
            [0.000000, 63.267593, 64.383271, 65.560133, 68.121427, 71.008456, 72.957073, 72.957073],
            [0.100000, 70.503476, 71.746753, 73.058212, 75.912441, 79.129658, 81.301137, 81.301137],
            [0.200000, 76.997217, 78.355007, 79.787258, 82.904376, 86.417916, 88.789399, 88.789399],
            [0.400000, 86.088455, 87.606562, 89.207922, 92.693086, 96.621477, 99.272967, 99.272967],
            [0.600000, 90.912377, 92.515550, 94.206642, 97.887095, 102.035612, 104.835676, 104.835676],
            [0.750000, 94.159247, 95.819677, 97.571165, 101.383063, 105.679741, 108.579808, 108.579808],
            [0.900000, 95.550763, 97.235732, 98.333795, 102.881334, 107.241510, 110.184435, 110.184435],
            [1.000000, 104.200000, 106.037491, 107.975750, 112.194133, 116.948991, 120.158309, 120.158309],
        ];

        return t[i][j];
    }

    /**
     * Table 1506 - Corrected net Thrust vs CN1 @ Mach 0 to 0.9 in 0.1 steps
     * n1_and_mach_on_thrust_table
     * @param i row index (CN1)
     * @param j mach
     * @returns Corrected net thrust (pounds of force)
     */
    static table1506(i: number, j: number) {
        const t = [
            [0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
            [20.000000, 0.091741, 0.057020, 0.031529, 0.014096, -0.017284, -0.037284, -0.057077, -0.205841, -0.315399, -0.488717],
            [25.000000, 0.142810, 0.072215, 0.038026, 0.020404, -0.009593, -0.026571, -0.024556, -0.151328, -0.266204, -0.439028],
            [30.000000, 0.189837, 0.082322, 0.04205, 0.026748, 0.017389, 0.003990, -0.026921, -0.056814, -0.081946, -0.369391],
            [35.000000, 0.262207, 0.126047, 0.077206, 0.045921, 0.024719, 0.006062, -0.0028121, -0.022800, -0.06972, -0.293631],
            [40.000000, 0.330230, 0.162757, 0.124088, 0.069579, 0.057905, 0.049621, 0.029790, 0.054284, 0.054218, -0.220630],
            [45.000000, 0.393293, 0.250096, 0.156707, 0.112419, 0.091418, 0.076757, 0.056090, 0.018509, -0.057520, -0.155120],
            [50.000000, 0.452337, 0.311066, 0.211353, 0.158174, 0.127429, 0.104915, 0.081171, 0.047419, -0.007399, -0.098474],
            [55.000000, 0.509468, 0.373568, 0.269961, 0.209106, 0.168650, 0.137223, 0.108383, 0.075660, 0.028704, -0.049469],
            [60.000000, 0.594614, 0.439955, 0.334629, 0.267477, 0.217773, 0.176899, 0.141404, 0.107148, 0.064556, -0.005036],
            [65.000000, 0.660035, 0.512604, 0.407151, 0.335055, 0.276928, 0.226669, 0.183627, 0.145850, 0.104441, 0.039012],
            [70.000000, 0.733601, 0.593506, 0.488571, 0.412623, 0.347163, 0.288210, 0.237559, 0.195142, 0.152485, 0.087269],
            [75.000000, 0.818693, 0.683880, 0.578756, 0.499514, 0.427939, 0.361604, 0.304241, 0.257197, 0.212005, 0.144042],
            [80.000000, 0.910344, 0.783795, 0.675982, 0.593166, 0.516644, 0.444822, 0.382689, 0.332384, 0.284867, 0.212679],
            [85.000000, 1.025165, 0.891823, 0.776548, 0.688692, 0.608128, 0.533210, 0.469351, 0.418690, 0.370870, 0.294907],
            [90.000000, 1.157049, 1.004695, 0.874400, 0.778466, 0.694251, 0.619011, 0.557581, 0.511153, 0.467149, 0.390203],
            [95.000000, 1.281333, 1.116993, 0.960774, 0.851733, 0.763455, 0.690890, 0.637136, 0.601322, 0.567588, 0.495167],
            [100.000000, 1.357935, 1.220844, 1.023864, 0.894234, 0.800352, 0.733488, 0.693684, 0.654691, 0.617963, 0.539115],
            [105.000000, 1.378826, 1.239626, 1.048498, 0.915750, 0.819609, 0.751137, 0.710375, 0.670444, 0.632832, 0.552086],
            [110.000000, 1.392754, 1.252148, 1.069322, 0.933937, 0.835886, 0.766054, 0.724483, 0.683759, 0.645400, 0.563051]
        ];

        return t[i][j];
    }
}

class FlightModel {
    static Cd0 = 0.0237;

    static wingSpan = 117.5;

    static wingArea = 1313.2;

    static wingEffcyFactor = 0.75;

    /**
     * Get lift coefficient at given conditions
     * @param weight in pounds
     * @param mach self-explanatory
     * @param delta pressure at the altitude divided by the pressure at sea level
     * @param loadFactor g-Force
     * @returns lift coefficient (Cl)
     */
    static getLiftCoefficient(weight: number, mach: number, delta: number, loadFactor = 1): number {
        return (weight * loadFactor) / (1481.4 * (mach ** 2) * delta * this.wingArea);
    }

    /**
     * Get drag coefficient at given conditions
     * TODO: support more mach numbers & flap and gear drag increments
     * @param weight in pounds
     * @param mach self-explanatory
     * @param delta pressure at the altitude divided by the pressure at sea level
     * @param spdBrkDeflected Whether speedbrake is deflected at half or not
     * @returns drag coefficient (Cd)
     */
    static getDragCoefficient(weight: number, mach: number, delta: number, spdBrkDeflected: boolean) : number {
        const Cl = this.getLiftCoefficient(weight, mach, delta);

        // For mach 0.78 only
        const cleanConfigDrag = (0.0384 * Cl ** 5) - (0.1385 * Cl ** 4) + (0.1953 * Cl ** 3) - (0.0532 * Cl ** 2) - (0.0052 * Cl) + 0.0259;

        const spdBrkIncrement = spdBrkDeflected ? 0.01 : 0;
        return cleanConfigDrag + spdBrkIncrement;
    }

    /**
     * Get drag at given conditions
     * @param weight in pounds
     * @param mach self-explanatory
     * @param delta pressure at the altitude divided by the pressure at sea level
     * @param spdBrkDeflected Whether speedbrake is deflected at half or not
     * @returns drag
     */
    static getDrag(weight: number, mach: number, delta: number, spdBrkDeflected: boolean): number {
        const Cd = this.getDragCoefficient(weight, mach, delta, spdBrkDeflected);
        return 1481.4 * (mach ** 2) * delta * this.wingArea * Cd;
    }

    /**
     * Gets acceleration factor for altitudes below troposphere
     * @param mach self-explanatory
     * @param temp actual temperature in Kelvin
     * @param stdTemp standard day temperature in Kelvin
     * @returns acceleration factor
     */
    static getAccelerationFactorBelowTropo(mach: number, temp: number, stdTemp: number): number {
        return 1 - (0.133184 * mach ** 2) * (stdTemp / temp);
    }

    /**
     * Gets acceleration factor for altitudes above troposphere
     * @param mach self-explanatory
     * @param flyingAtConstantMach if aircraft is flying at a constant mach
     * @returns acceleration factor
     */
    static getAccelerationFactorAboveTropo(mach: number, flyingAtConstantMach: boolean): number {
        if (flyingAtConstantMach) {
            return 1;
        }

        const phi = (((1 + 0.2 * mach ** 2) ** 3.5) - 1) / ((0.7 * mach ** 2) * (1 + 0.2 * mach ** 2) ** 2.5);
        return 1 + (0.7 * mach ** 2) * phi;
    }

    static getConstantThrustPathAngle(
        thrust: number,
        weight: number,
        mach: number,
        drag: number,
        temp: number,
        stdTemp: number,
        altitude: number,
        tropoAlt: number,
        flyingAtConstantMach: boolean,
    ): number {
        const accelFactor = altitude >= tropoAlt ? this.getAccelerationFactorAboveTropo(mach, flyingAtConstantMach) : this.getAccelerationFactorBelowTropo(mach, temp, stdTemp);
        return Math.asin(((thrust - drag) / weight) / accelFactor);
    }

    /**
     * Gets distance required to accelerate/decelerate
     * @param thrust 
     * @param drag 
     * @param weight in pounds
     * @param initialSpeed 
     * @param targetSpeed
     * @returns distance to accel/decel
     */
    static getAccelerationDistance(
        thrust: number,
        drag: number,
        weight: number,
        initialSpeed: number,
        targetSpeed: number,
    ): number {
        const force = thrust - drag;
        const accel = force / weight; // TODO: Check units
        const timeToAccel = (targetSpeed - initialSpeed) / accel;
        const distanceToAccel = (initialSpeed * timeToAccel) + (0.5 * accel * (timeToAccel ** 2)); // TODO: Check units
        return distanceToAccel;
    }
}

enum VnavState {
    NONE,
    REPRESSURIZATION,
    IDLE,
    GEOMETRIC
}

interface Constraint {
    upperConstraint: number;
    lowerConstraint: number;
    isAtConstraint: boolean;
    hasConstraint: boolean;
}

interface AtConstraint {
    index: number;
    altitude: number;
}
