using Rhino;
using Rhino.Geometry;
using Rhino.DocObjects;
using Rhino.Collections;

using GH_IO;
using GH_IO.Serialization;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

using System;
using System.IO;
using System.Xml;
using System.Xml.Linq;
using System.Linq;
using System.Data;
using System.Drawing;
using System.Reflection;
using System.Collections;
using System.Windows.Forms;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using KPlankton;
using KangarooSolver;

#region padding (this ensures the line number of this file match with those in the code editor of the C# Script component










#endregion

public partial class Interlace : GH_ScriptInstance
{
    #region Do_not_modify_this_region
    private void Print(string text) { }
    private void Print(string format, params object[] args) { }
    private void Reflect(object obj) { }
    private void Reflect(object obj, string methodName) { }
    public override void InvokeRunScript(IGH_Component owner, object rhinoDocument, int iteration, List<object> inputs, IGH_DataAccess DA) { }
    public RhinoDoc RhinoDocument;
    public GH_Document GrasshopperDocument;
    public IGH_Component Component;
    public int Iteration;
    #endregion


    private void RunScript(Mesh M, double S, bool Reset, bool On, object Grab, ref object adM, ref object L, ref object N)
    {
        // <Custom code> 
        if(Reset || GoalList == null)
        {
            curves = new List<Polyline>();
            initWeaveMesh = ToInterlaceMesh(M, S, out curves, out nodeCurveID);
            
            //kangaroo section
            GoalList = new List<IGoal>();
            PS = new PhysicalSystem();
            List<int>[] kParticleIds = new List<int>[nodeCurveID.Length];
            List<int> collisionIDs = new List<int>();
            int id = -1;
            for(int i = 0; i < nodeCurveID.Length; i++)
            {
                List<int> ids = new List<int>();
                foreach (int j in nodeCurveID[i])
                {
                    //NOTE:
                    //this adding standard sequence (0,1,2,3,4....) to kParticles not using nodeCurveID directly.
                    id++;
                    collisionIDs.Add(id);
                    ids.Add(id);
                }
                kParticleIds[i] = ids;
            }
            //add particle and goal
            for(int i = 0; i < curves.Count; i++)
            {
                for(int j = 0; j < curves[i].Count; j++)
                {
                    PS.AddParticle(curves[i][j], 0.0);
                    if(j != 0)
                    {
                        IGoal spring = new KangarooSolver.Goals.Spring(kParticleIds[i][j - 1], kParticleIds[i][j], curves[i].SegmentAt(j - 1).Length, 30);
                        GoalList.Add(spring);
                    }
                    if(j < curves[i].Count - 2)
                    {
                        IGoal angle = new KangarooSolver.Goals.Angle(30.0, 0.0, kParticleIds[i][j], kParticleIds[i][j + 1], kParticleIds[i][j + 1], kParticleIds[i][j + 2]);
                        GoalList.Add(angle);
                    }
                    
                    //if(curves[i].Count - 5 >= 0 || j <= curves[i].Count - 4) //012345
                    //{
                        //GoalObject continuity = new CurveContinuityID(new List<int> { kParticleIds[i][j], kParticleIds[i][j+1], kParticleIds[i][j+2], kParticleIds[i][j+3], kParticleIds[i][4] }, 10);
                        //GoalList.Add(continuity);
                    //}
                    if(j == 0 || j == curves[i].Count -1)
                    {
                        IGoal anchor = new KangarooSolver.Goals.Anchor(kParticleIds[i][j], curves[i][j], 5);
                        GoalList.Add(anchor);
                    }
                    //IGoal colinear = new KangarooSolver.Goals.CoLinear(kParticleIds[i], 0.01);
                    //GoalList.Add(colinear);
                }
            }
            for (int i = 0; i < nodeCurveID.Length; i++)
            {
                for (int j = 0; j < nodeCurveID[i].Count; j++)
                {
                    //Print(kParticleIds[i][j].ToString() + "," + nodeCurveID[i][j].ToString());
                    IGoal slideOnCrv = new PointSlideOnCurve(kParticleIds[i][j], kParticleIds[nodeCurveID[i][j]], 0.2);
                    GoalList.Add(slideOnCrv);
                }
            }
            IGoal collision = new SphereCollideID(collisionIDs, 0.2, 1);
            /*
            GoalList.Add(collision);
            if (Grab != null)
                GoalList.Add((IGoal)Grab);
                */
        }

        if (On)
        {
            if(PS.GetIterations() < 600)
                PS.Step(GoalList, true, 0.001);
            int id = -1;
            for(int i = 0; i < curves.Count; i++)
            {
                for(int j = 0; j < curves[i].Count; j++)
                {
                    id++;
                    curves[i][j] = PS.GetPosition(id);
                }
            }
        }
        else
        {
            curves = new List<Polyline>();
            initWeaveMesh = ToInterlaceMesh(M, S, out curves, out nodeCurveID);
        }
        adM = initWeaveMesh;
        L = curves;
        //N = PS.GetOutput(GoalList);
        #region Description
        Component.Params.Input[0].Description = "Mesh";
        Component.Params.Input[1].Description = "Amplitude";
        #endregion
        // </Custom code> 
    }

    // <Custom additional code> 
    Mesh initWeaveMesh;
    List<int>[] nodeCurveID;
    List<Polyline> curves;
    KangarooSolver.PhysicalSystem PS = new KangarooSolver.PhysicalSystem();
    List<IGoal> GoalList = new List<IGoal>();
    public static Mesh ToInterlaceMesh(Mesh M, double Amplitude, out List<Polyline> WeaveCurves, out List<int>[] NodeBelongCurveID)
    {
        KPlanktonMesh kM = M.ToKPlanktonMesh();
        List<Curve> curves = new List<Curve>();
        List<Vector3d> normals = new List<Vector3d>();
        WeaveCurves = new List<Polyline>();
        Mesh wM = new Mesh();
        for(int i = 0; i < kM.Halfedges.Count; i+=2)
        {
            int s = kM.Halfedges[i].StartVertex;
            int e = kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)].StartVertex;
            
            Point3d pS = kM.Vertices[s].ToPoint3d();
            Point3d pE = kM.Vertices[e].ToPoint3d();

            Vector3d nS = new Vector3d(kM.Vertices.GetNormal(s).ToVector3f());
            Vector3d nE = new Vector3d(kM.Vertices.GetNormal(e).ToVector3f());
            
            Point3d newV = (pS + pE) / 2;
            Vector3d newN = (nS + nE); newN.Unitize();
            
            wM.Vertices.Add(newV);
            normals.Add(newN);
        }

        for(int i = 0; i < kM.Faces.Count; i++)
        {
            List<int> newFaces = new List<int>();
            int[] faceEdges = kM.Faces.GetHalfedges(i);
            foreach(int e in faceEdges)
            {
                if (e != 0) //greater than 0
                {
                    if (e % 2 == 0) //first edge
                        newFaces.Add(e / 2);
                    else //add index from pair number
                        newFaces.Add(kM.Halfedges.GetPairHalfedge(e) / 2);
                }
                else
                    newFaces.Add(e);
            }
            wM.Faces.AddFace(newFaces[0], newFaces[1], newFaces[2]);
            Point3d v0 = new Point3d(wM.Vertices[newFaces[0]]);
            Point3d v1 = new Point3d(wM.Vertices[newFaces[1]]);
            Point3d v2 = new Point3d(wM.Vertices[newFaces[2]]);

            curves.Add(new LineCurve(v0, v1 + normals[newFaces[1]] * Amplitude));
            curves.Add(new LineCurve(v1, v2 + normals[newFaces[2]] * Amplitude));
            curves.Add(new LineCurve(v2, v0 + normals[newFaces[0]] * Amplitude));
        }
        
        //SlideOnCurve behavior:
        //let node to slide along on cuvre through pair to closet curve and keep it on curve.
        //todo: dataset is [node][curve id]

        Curve[] jointCurves = Curve.JoinCurves(curves); //combine nodes to new curve, but any better solution?
        NodeBelongCurveID = new List<int>[jointCurves.Length];
        //NOTE: NodeBelongCurveID
        //[curve index]: node interlace to another curve index.......
        for (int i = 0; i < jointCurves.Length; i ++)
        {
            Polyline pCurve;
            jointCurves[i].TryGetPolyline(out pCurve);
            //======== pull weave cuvre back to mesh for simulation purpose
            
            List<int> ids = new List<int>();
            for (int j = 0; j < pCurve.Count; j++)
            {
                double distance = 10000.0;
                int closestID = 0;
                Point3d v = new Point3d(wM.Vertices[0]);
                double pullT = 10000.0;
                for (int k = 0; k < wM.Vertices.Count; k++)
                {
                    if (pullT > pCurve[j].DistanceTo(new Point3d(wM.Vertices[k])))
                    {
                        pullT = pCurve[j].DistanceTo(new Point3d(wM.Vertices[k]));
                        v = new Point3d(wM.Vertices[k]);
                    }
                }
                pCurve[j] = v;
                for (int k = 0; k < jointCurves.Length; k ++)
                {
                    if (k != i)
                    {
                        // find closet point from all polyline nodes
                        double t;
                        PolylineCurve polyline = jointCurves[k] as PolylineCurve;
                        polyline.ClosestPoint(pCurve[j], out t);
                        double testDistance = jointCurves[k].PointAt(t).DistanceTo(pCurve[j]);
                        if (testDistance < distance)
                        {
                            closestID = k;
                            distance = testDistance;
                        }
                    }
                }
                ids.Add(closestID); // pre add to array of list
            }

            NodeBelongCurveID[i] = ids;
            WeaveCurves.Add(pCurve);
        }
        return wM;
    }
    public class PointSlideOnCurve : GoalObject
    {
        public double strength;
        public PointSlideOnCurve(int NodeID, List<int> TargetCurveID, double k)
        {
            int L = TargetCurveID.Count + 1;
            int[] Points = new int[L];
            Points[0] = NodeID;
            for(int i = 0; i < TargetCurveID.Count; i++)
                Points[i + 1] = TargetCurveID[i];
            PIndex = Points;
            Move = new Vector3d[L];
            Weighting = new double[L];
            for (int j = 0; j < L; j++)
                Weighting[j] = k;
            strength = k;
        }
        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            int L = PIndex.Length;
            for(int i = 0; i < L; i++)
            {
                Move[i] = Vector3d.Zero;
                Weighting[i] = strength;
            }

            int[] curveIDs = new int[L - 1];
            for (int i = 1; i < L; i++)
                curveIDs[i - 1] = PIndex[i];
            List<Point3d> pts = new List<Point3d>();
            foreach(int i in curveIDs)
            {
                pts.Add(p[i].Position);
            }
            PolylineCurve polyline = new PolylineCurve(pts);
            double t;
            polyline.ClosestPoint(p[PIndex[0]].Position, out t);
            Vector3d toClosetPt = polyline.PointAt(t) - p[PIndex[0]].Position;
            toClosetPt.Unitize();
            Move[0] = toClosetPt * Weighting[0];
        }
        public override object Output(List<KangarooSolver.Particle> p)
        {
            int L = PIndex.Length;
            List<Point3d> pts = new List<Point3d>();
            for (int i = 1; i < L; i++)
                pts.Add(p[PIndex[i]].Position);
            Polyline polyline = new Polyline(pts);
            return polyline;
        }
    }
    public class SphereCollideID : GoalObject
    {
        public double Strength;
        public double SqDiam;
        public double Diam;

        public SphereCollideID(List<int> V, double r, double k)
        {
            int L = V.Count;
            PIndex = V.ToArray();
            Move = new Vector3d[L];
            Weighting = new double[L];
            for (int i = 0; i < L; i++)
            {
                Weighting[i] = k;
            }
            Diam = r + r;
            SqDiam = Diam * Diam;
            Strength = k;
        }

        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            int L = PIndex.Length;
            double[] Xcoord = new double[L];
            for (int i = 0; i < L; i++)
            {
                Xcoord[i] = p[PIndex[i]].Position.X;
            }
            Array.Sort(Xcoord, PIndex);

            for (int i = 0; i < L; i++)
            {
                Move[i] = Vector3d.Zero;
                Weighting[i] = 0;
            }

            for (int i = 0; i < (PIndex.Length - 1); i++)
            {
                for (int j = 1; (i + j) < PIndex.Length; j++)
                {
                    int k = i + j;
                    Vector3d Separation = p[PIndex[k]].Position - p[PIndex[i]].Position;
                    if (Separation.X < Diam)
                    {
                        if (Separation.SquareLength < SqDiam)
                        {
                            double LengthNow = Separation.Length;
                            double stretchfactor = 1.0 - Diam / LengthNow;
                            Vector3d SpringMove = 0.5 * Separation * stretchfactor;
                            Move[i] += SpringMove;
                            Move[k] -= SpringMove;
                            Weighting[i] = Strength;
                            Weighting[k] = Strength;
                        }
                    }

                    else { break; }
                }
            }
        }
    }
    //NOTE 0407: 
    //still need to debug continuity function in P2 & Move[2] will cause error (null point) in solver.
    //remain not implement and question necessity in bend behavior.
    public class CurveContinuityID: GoalObject
    {
        public CurveContinuityID(List<int> ids, double k)
        {
            PIndex = ids.ToArray();
            Move = new Vector3d[5];
            Weighting = new double[] { k, k, k, k, k };
        }

        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            Point3d P0 = p[PIndex[0]].Position;
            Point3d P1 = p[PIndex[1]].Position;
            Point3d P2 = p[PIndex[2]].Position;
            Point3d P3 = p[PIndex[3]].Position;
            Point3d P4 = p[PIndex[4]].Position;
            Vector3d V31 = P3 - P1;
            Vector3d V21 = P2 - P1;
            double length = V31.Length;
            V31.Unitize();
            double num = V21 * V31;
            Move[2] = -0.25 * (V21 - num * V31);
            double num2 = num / length;
            Move[1] = (1.0 - num2) * - Move[2];
            Move[3] = num2 * - Move[2];
            double length2 = V21.Length;
            double num3 = P2.DistanceTo(P3);
            Vector3d V10 = P1 - P0;
            num = V10 * V31;
            Vector3d val4 = V10 - V31 * num;
            double length3 = val4.Length;
            Vector3d val5 = P3 - P4;
            num = val5 * V31;
            Vector3d val6 = val5 - V31 * num;
            double length4 = val6.Length;
            double num4 = length2 * length2 / (2.0 * num3 * num3) + length3 / (2.0 * length4);
            double num5 = Math.Sqrt(num4 * num3 * num3);
            double num6 = Math.Sqrt(length2 * length2 / num4);
            double num7 = num4 * length4;
            double num8 = length3 / num4;
            double num9 = length2 - num5;
            double num10 = num3 - num6;
            double num11 = length3 - num7;
            double num12 = length4 - num8;
            V10.Unitize();
            Move[0] = 0.1 * V10 * num11;
            Move[1] -= 0.5 * Move[0];
            val5.Unitize();
            Move[4] = 0.1 * val5 * num12;
            Move[3] -= 0.5 * Move[4];
            Vector3d val7 = 0.1 * V31 * num9;
            Move[1] += val7;
            Move[2] -= val7;
            val7 = 0.1 * V31 * num10;
            Move[1] -= val7;
            Move[2] += val7;
        }
    }
    // </Custom additional code>  
}
