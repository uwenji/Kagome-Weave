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

public partial class InterlaceEnergy : GH_ScriptInstance
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


    private void RunScript(Mesh M, double S, bool Reset, bool On, double LRatio, double Anchor, ref object adM, ref object L, ref object ID)
    {
        // <Custom code> 
        if(Reset || GoalList == null)
        {
            curves = new List<Polyline>();
            initWeaveMesh = ToInterlaceMesh(M, 0.5, out curves, out nodePhase);
            KPlanktonMesh kM = initWeaveMesh.ToKPlanktonMesh();
            //kangaroo section
            GoalList = new List<IGoal>();
            PS = new PhysicalSystem();
            kParticleIds = new List<int>[curves.Count];
            for (int i = 0; i < curves.Count; i++)
            {
                kParticleIds[i] = new List<int>();
                for (int j = 0; j < curves[i].Count; j++)
                    kParticleIds[i].Add(-1);
            }

            //prepare kangaroo particles data 
            for (int i = 0; i < kM.Vertices.Count; i++)
            {
                Point3d v = kM.Vertices[i].ToPoint3d();
                PS.AddParticle(v, 0.0);
                for (int j = 0; j < curves.Count; j++)
                {
                    for (int k = 0; k < curves[j].Count; k++)
                    {
                        Point3d node = curves[j][k];
                        if (v.DistanceTo(node) <= 0.001)
                            kParticleIds[j][k] = i;
                    }
                }
            }
            
            //add particle and goal 
            for (int i = 0; i < kParticleIds.Length; i++)
            {
                for(int j = 0; j < kParticleIds[i].Count; j++)
                {
                    if(j < kParticleIds[i].Count - 1)//spring
                    {
                        double l = curves[i].SegmentAt(j).Length;
                        IGoal spring = new KangarooSolver.Goals.Spring(kParticleIds[i][j], kParticleIds[i][j + 1], l * LRatio, 10);
                        GoalList.Add(spring);
                    }

                    if(j < kParticleIds[i].Count - 2)//bend
                    {
                        IGoal bend = new KangarooSolver.Goals.Bend(50.0, 0.0, kParticleIds[i][j], kParticleIds[i][j + 1], kParticleIds[i][j + 2]);
                        GoalList.Add(bend);
                    }

                    if(j == 0 || j == kParticleIds[i].Count -1)//anchor
                    {
                        IGoal pAnchor = new PlasticAnchorID(kParticleIds[i][j], curves[i][j], 2, Anchor);
                        GoalList.Add(pAnchor);
                    }
                }
            }
            //equalAngle
            for(int i = 0; i < kM.Vertices.Count; i++)
            {
                int[] incomings = kM.Vertices.GetIncomingHalfedges(i);
                if(incomings.Length > 2)
                {
                    int o = i;
                    int a = kM.Halfedges[incomings[0]].StartVertex;
                    int b = kM.Halfedges[incomings[1]].StartVertex;
                    int c = kM.Halfedges[incomings[2]].StartVertex;
                    int d = kM.Halfedges[incomings[3]].StartVertex;
                    List<int> lineA = new List<int> { o, a, o, d };
                    List<int> lineB = new List<int> { o, b, o, c };
                    List<int> lineC = new List<int> { o, b, o, d };
                    List<int> lineD = new List<int> { o, c, o, a };
                    IGoal eualAngle1 = new EqualAngleID(lineA, lineB, 1.0);
                    IGoal eualAngle2 = new EqualAngleID(lineC, lineD, 1.0);
                    GoalList.Add(eualAngle1);
                    GoalList.Add(eualAngle2);
                }
            }
        }

        if (On)
        {
            if(PS.GetIterations() < 1000)
                PS.Step(GoalList, true, 0.001);
            for (int i = 0; i < initWeaveMesh.Vertices.Count; i++)
                initWeaveMesh.Vertices[i] = new Point3f((float)PS.GetPosition(i).X, (float)PS.GetPosition(i).Y, (float)PS.GetPosition(i).Z);
            for (int i = 0; i < kParticleIds.Length ; i++)
            {
                for (int j = 0; j < kParticleIds[i].Count; j++)
                {
                    Vector3d normal = new Vector3d(initWeaveMesh.Normals[kParticleIds[i][j]]);
                    if (nodePhase[i][j])
                        curves[i][j] = PS.GetPosition(kParticleIds[i][j]) + normal * 0.5;
                    else
                        curves[i][j] = PS.GetPosition(kParticleIds[i][j]);
                }
            }

            
        }
        else
        {
            curves = new List<Polyline>();
            initWeaveMesh = ToInterlaceMesh(M, 0.5, out curves, out nodePhase);
        }

        DataTree<int> tree = new DataTree<int>();
        for (int i = 0; i < kParticleIds.Length; i++)
            tree.AddRange(kParticleIds[i], new GH_Path(i));

        adM = initWeaveMesh;
        L = curves;
        ID = tree;
        #region Description
        Component.Params.Input[0].Description = "Mesh";
        Component.Params.Input[2].Description = "Relaxation Length";
        Component.Params.Input[3].Description = "Anchor Strength";
        #endregion
        // </Custom code> 
    }

    // <Custom additional code> 
    Mesh initWeaveMesh;
    List<Polyline> curves;
    KangarooSolver.PhysicalSystem PS = new KangarooSolver.PhysicalSystem();
    List<IGoal> GoalList = new List<IGoal>();
    List<int>[] kParticleIds;
    List<bool>[] nodePhase;
    public static Mesh ToInterlaceMesh(Mesh M, double Amplitude, out List<Polyline> WeaveCurves, out List<bool>[] Phase)
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
        wM.RebuildNormals();
        wM.UnifyNormals(true);

        Curve[] jointCurves = Curve.JoinCurves(curves); //combine nodes to new curve, but any better solution?
        Phase = new List<bool>[jointCurves.Length];

        for (int i = 0; i < jointCurves.Length; i ++)
        {
            Polyline pCurve;
            jointCurves[i].TryGetPolyline(out pCurve);
            //======== pull weave cuvre back to mesh for simulation purpose
            Phase[i] = new List<bool>();
            List<int> ids = new List<int>();
            for (int j = 0; j < pCurve.Count; j++)
            {
                Point3d v = new Point3d(wM.Vertices[0]);
                Vector3d n = new Vector3d(wM.Normals[i]);
                double pullT = 10000.0;
                for (int k = 0; k < wM.Vertices.Count; k++)
                {
                    if (pullT > pCurve[j].DistanceTo(new Point3d(wM.Vertices[k])))
                    {
                        pullT = pCurve[j].DistanceTo(new Point3d(wM.Vertices[k]));
                        v = new Point3d(wM.Vertices[k]);
                    }
                }
                if ((v + n * Amplitude).DistanceTo(pCurve[j]) < Amplitude)
                    Phase[i].Add(true);
                else
                    Phase[i].Add(false);
                pCurve[j] = v;
            }

            WeaveCurves.Add(pCurve);
        }
        return wM;
    }
    public class PlasticAnchorID : GoalObject
    {
        public double limit;
        public Point3d location;
        public PlasticAnchorID(int NodeID, Point3d Target, double r, double k)
        {
            PIndex = new int[1] { NodeID };
            Move = new Vector3d[1];
            Weighting = new double[1] { k};
            location = Target;
            limit = r;
        }
        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            Point3d position = p[PIndex[0]].Position;
            Vector3d vec = location - position;
            Move[0] = vec;
            double l = vec.Length - limit;
            if(l > 0.0)
            {
                vec.Unitize();
                vec *= l;
                location -= vec;
                Move[0] -= vec;
            }
        }
    }
    public class EqualAngleID : GoalObject
    {
        public double stiffness;
        public EqualAngleID(List<int> indicesA, List<int> indicesB,double k) 
        {
            int l = indicesA.Count / 2;
            int[] points = new int[l*4];
            Move = new Vector3d[l*4];
            Weighting = new double[l*4];

            for (int i = 0; i < l; i++)
            {
                Move[i] = Move[i+l] = Move[i+l*2] = Move[i*3] = Vector3d.Zero;
                Weighting[i] = Weighting[i+l] = Weighting[i+l*2] = Weighting[i+l*3] = k;

                points[i] = indicesA[i];
                points[i + l] = indicesA[i + 1];
                points[i + l * 2] = indicesB[i];
                points[i + l * 3] = indicesB[i + 1];
            }
            PIndex = points;
            stiffness = k;
        }
        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            int l = PIndex.Length / 4;
            Point3d[] ptsA0 = new Point3d[l];
            Point3d[] ptsA1 = new Point3d[l];
            Point3d[] ptsB0 = new Point3d[l];
            Point3d[] ptsB1 = new Point3d[l];
            Vector3d[] vecA = new Vector3d[l];
            Vector3d[] vecB = new Vector3d[l];
            double[] angles = new double[l];
            double averageAngle = 0.0;
            for(int i = 0; i < l; i++)
            {
                ptsA0[i] = p[PIndex[i]].Position;
                ptsA1[i] = p[PIndex[i + l]].Position;
                ptsB0[i] = p[PIndex[i + l * 2]].Position;
                ptsB1[i] = p[PIndex[i + l * 3]].Position;
                vecA[i] = ptsA1[i] - ptsA0[i];
                vecB[i] = ptsB1[i] - ptsB0[i];
                angles[i] = Vector3d.VectorAngle(vecA[i], vecB[i]);
                averageAngle += angles[i];
            }
            averageAngle = averageAngle / l;
            for(int i = 0; i < l; i++)
            {
                double sinVal = Math.Sin(angles[i] - averageAngle);
                Vector3d v1 = vecA[i] + vecB[i];
                double length = v1.Length;
                double LA = sinVal / (vecA[i].Length * length);
                double LB = sinVal / (vecB[i].Length * length);
                Vector3d crossAB = Vector3d.CrossProduct(vecA[i], vecB[i]);
                Vector3d crossA_AB = Vector3d.CrossProduct(crossAB, vecA[i]);
                Vector3d crossB_AB = Vector3d.CrossProduct(crossAB, vecB[i]);
                crossA_AB *= LA * 0.5;
                crossB_AB *= LB * 0.5;
                Move[i] = crossA_AB;
                Move[i + l] = -crossA_AB;
                Move[i + l * 2] = crossB_AB;
                Move[i + l * 3] = -crossB_AB;
            }
        }
    }
    // </Custom additional code>  
}
