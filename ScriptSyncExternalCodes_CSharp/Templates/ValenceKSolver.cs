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

public partial class ValenceKSolver : GH_ScriptInstance
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


    private void RunScript(Mesh M, int desired, DataTree<object> Vertices, DataTree<int> vCount, List<string> Surgery, bool Append, bool Reset, bool On, bool Operate, object Grab,
        ref object adM, ref object I, ref object dEdges, ref object KM)
    {
        // <Custom code> 
        #region Kangaroo
        if (Reset || GoalList == null)
        {
            //init setup values
            displayLabs = new List<int>();
            displayGraphes = new List<int>();
            displayEdges = new List<Line>();
            storedID = new List<int>();
            GoalList = new List<IGoal>();
            PS = new PhysicalSystem();
            length = 0.0; //for relaxe mesh
            adding = -1;
            operation = 0;
            kM = M.ToKPlanktonMesh();
            // set general mesh length
            foreach (KPlanktonHalfedge e in kM.Halfedges)
                length += kM.Vertices[e.StartVertex].ToPoint3d().DistanceTo(kM.Vertices[kM.Halfedges[e.NextHalfedge].StartVertex].ToPoint3d());

            length /= kM.Halfedges.Count;
            operation = 0;
            //prestored vertices index
            stored = -1;
        }
        if(Append && adding < Vertices.BranchCount - 1)
        {
            double t = 0.0001; //tolerance
            adding++;
            PS = new PhysicalSystem();

            if (Vertices.Branches[adding][0] is Rhino.Geometry.Point3d)
            {
                for (int j = 0; j < kM.Vertices.Count; j++)
                {
                    Point3d v = ((Rhino.Geometry.Point3d)Vertices.Branches[adding][0]);
                    if (kM.Vertices[j].ToPoint3d().DistanceTo(v) <= t)
                        stored = j;
                }
            }
            else
                stored = Convert.ToInt32(Vertices.Branches[adding][0]);
            
            storedID.Add(stored);
            //region define
            List<int> flipEdges = InputValence(kM, stored, vCount.Branches[adding]);
            
            for (int i = 0; i < vCount.Branches[adding].Count; i++)
            {
                bool type;
                if (vCount.Branches[adding][i] < 0)
                    type = false;
                else
                    type = true;
                SearchRegion(kM, flipEdges[i], type);
            }
            
            List<int> collisionVertices = new List<int>();
            for(int i = 0; i < kM.Vertices.Count; i ++)
            {
                PS.AddParticle(kM.Vertices[i].ToPoint3d(), 0.0);
                collisionVertices.Add(i);
            }

            //spring & hinge
            for(int i = 0; i < kM.Halfedges.Count; i+=2)
            {
                KPlanktonHalfedge e = kM.Halfedges[i];
                KPlanktonHalfedge ePair = kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)];
                IGoal spring = new KangarooSolver.Goals.Spring(e.StartVertex, ePair.StartVertex, length, 1.0);
                if (!kM.Halfedges.IsBoundary(i))
                {
                    IGoal hinge = new KangarooSolver.Goals.Hinge(e.StartVertex, ePair.StartVertex, kM.Halfedges[e.PrevHalfedge].StartVertex, kM.Halfedges[ePair.PrevHalfedge].StartVertex, 0.0, 1.0);
                    GoalList.Add(hinge);
                }
                GoalList.Add(spring);
            }
            IGoal collision = new SphereCollideID(collisionVertices, length / 2, 3.0);
            GoalList.Add(collision);

            //NOTE: some issue by replacment new grab goal in gh work
            if (Grab != null)
                GoalList.Add((IGoal)Grab);
        }
        if (Operate && operation < 1)
        {
            operation++;
            List<string[]> edgeSurgeries = new List<string[]>();
            foreach (string str in Surgery)
                edgeSurgeries.Add(str.Split(':'));

            foreach (string[] str in edgeSurgeries)
            {
                if (str[0] == "S")
                    kM.Halfedges.TriangleSplitEdge(Convert.ToInt32(str[1]));

                if (str[0] == "F")
                    kM.Halfedges.FlipEdge(Convert.ToInt32(str[1]));

                if (str[0] == "A")
                {
                    string[] sF = str[1].Split(',');
                    int okay = kM.Faces.AddFace(Convert.ToInt32(sF[0]), Convert.ToInt32(sF[1]), Convert.ToInt32(sF[2]));
                    Print(okay.ToString());
                }
                    
            }

            //=================
            adding++;
            PS = new PhysicalSystem();

            List<int> collisionVertices = new List<int>();
            for (int i = 0; i < kM.Vertices.Count; i++)
            {
                PS.AddParticle(kM.Vertices[i].ToPoint3d(), 0.0);
                collisionVertices.Add(i);
            }

            //spring & hinge
            for (int i = 0; i < kM.Halfedges.Count; i += 2)
            {
                KPlanktonHalfedge e = kM.Halfedges[i];
                KPlanktonHalfedge ePair = kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)];
                IGoal spring = new KangarooSolver.Goals.Spring(e.StartVertex, ePair.StartVertex, length, 1.0);
                if (!kM.Halfedges.IsBoundary(i))
                {
                    IGoal hinge = new KangarooSolver.Goals.Hinge(e.StartVertex, ePair.StartVertex, kM.Halfedges[e.PrevHalfedge].StartVertex, kM.Halfedges[ePair.PrevHalfedge].StartVertex, 0.0, 1.0);
                    GoalList.Add(hinge);
                }
                GoalList.Add(spring);
            }
            IGoal collision = new SphereCollideID(collisionVertices, length / 2, 3.0);
            GoalList.Add(collision);

            //NOTE: some issue by replacment new grab goal in gh work
            if (Grab != null)
                GoalList.Add((IGoal)Grab);

        }
        if (On && storedID.Count != 0)
        {
            PS.Step(GoalList, true, 0.001);

            for (int i = 0; i < kM.Vertices.Count; i++)
            {
                Point3d kPt = PS.GetPosition(i);
                kM.Vertices[i].X = (float)kPt.X;
                kM.Vertices[i].Y = (float)kPt.Y;
                kM.Vertices[i].Z = (float)kPt.Z;
            }
        }
        
        #endregion
        
        //display desired edge
        if(kM.Vertices.Count > 0)
        {
            desiredEdges = new List<Line>();

            int[] halfs = kM.Vertices.GetIncomingHalfedges(desired);
            for (int i = 0; i < halfs.Length; i++)
            {
                Point3d sDecr = kM.Vertices[kM.Halfedges[halfs[i]].StartVertex].ToPoint3d();
                Point3d eDecr = kM.Vertices[kM.Halfedges[kM.Halfedges.GetPairHalfedge(halfs[i])].StartVertex].ToPoint3d();
                desiredEdges.Add(new Line(sDecr, eDecr));

                Point3d sIncr = kM.Vertices[kM.Halfedges[kM.Halfedges[halfs[i]].PrevHalfedge].StartVertex].ToPoint3d();
                desiredEdges.Add(new Line(sIncr, sDecr));
            }
        }
        
        //output section
        
        Mesh rM = kM.ToRhinoMesh();
        adM = rM;
        I = PS.GetIterations();
        dEdges = desiredEdges;
        KM = kM;

        #region component description
        Component.Message = "ValenceTool";
        Component.Params.Input[0].Description = "Mesh";
        Component.Params.Input[1].Description = "Desired vertex number";
        Component.Params.Input[2].Description = "Singularity point";
        Component.Params.Input[3].Description = "Valence number";
        Component.Params.Input[4].Description = "Split or Flip edge for fix singularity.\ne.g. S:100, F:99";
        Component.Params.Output[1].Description = "Iteration count";
        Component.Params.Output[2].Description = "Adjustment Mesh";
        #endregion
        // </Custom code> 
    }

    // <Custom additional code> 
    KPlanktonMesh kM;
    KangarooSolver.PhysicalSystem PS = new KangarooSolver.PhysicalSystem();
    List<IGoal> GoalList = new List<IGoal>();
    int operation = -1;
    #region Visualize edges
    List<Line> displayEdges = new List<Line>();
    List<int> displayGraphes = new List<int>();
    List<int> displayLabs = new List<int>();
    List<int> storedID = new List<int>();
    List<Line> desiredEdges = new List<Line>();
    int adding;
    int stored;
    double length;
    public override void DrawViewportWires(IGH_PreviewArgs args)
    {
        base.DrawViewportWires(args);
        for (int i = 0; i < storedID.Count; i++)
        {
            args.Display.DrawDot(kM.Vertices[storedID[i]].ToPoint3d() + new Point3d(0, 0, 1.0), i.ToString(), System.Drawing.Color.Azure, System.Drawing.Color.White);
        }

        Plane plane;
        args.Viewport.GetFrustumFarPlane(out plane);

        for (int i = 0; i < displayEdges.Count; i++)
        {
            plane.Origin = (displayEdges[i].From + displayEdges[i].To) * 0.5;
            args.Display.DrawLine(displayEdges[i], System.Drawing.Color.DarkOrange, 3);
            Rhino.Display.Text3d drawText = new Rhino.Display.Text3d(displayLabs[i].ToString(), plane, 2);
            args.Display.Draw3dText(drawText, System.Drawing.Color.HotPink);
            drawText.Dispose();
        }
    }
    #endregion

    //Valence adjustment methodology
    public List<int> InputValence(KPlanktonMesh kM, int V, List<int> E)
    {
        /* -----------------------------------
        type 0 = increasing(7,8,9), 1 = decreasing(5,4,3)
        "//" represent selected edge, "*" is vertex, "<=" is the halfedge direction
        [decreasing]    [increasing]

            /\              ^     
           /  \            /||\   
          /    \          / || \  
        *<=======        *  ||  |  
          \    /          \ || /  
           \  /            \||/    
        --------------------------------------*/
        List<int> Edges = new List<int>();

        int[] halfs = kM.Vertices.GetIncomingHalfedges(V);
        for (int i = 0; i < halfs.Length; i++)
        {
            int sDecr = kM.Halfedges[halfs[i]].StartVertex;
            int eDecr = kM.Halfedges[kM.Halfedges.GetPairHalfedge(halfs[i])].StartVertex;
            displayGraphes.Add(sDecr);
            displayGraphes.Add(eDecr);
            displayLabs.Add(-i - 1);

            int sIncr = kM.Halfedges[kM.Halfedges[halfs[i]].PrevHalfedge].StartVertex;
            displayGraphes.Add(sIncr);
            displayLabs.Add(i + 1);
            foreach (int k in E)
            {
                if (k == (-i - 1))
                    Edges.Add(halfs[i]);
                if (k == (i + 1))
                    Edges.Add(kM.Halfedges[halfs[i]].PrevHalfedge);
            }
        }
        return Edges;
    }

    public bool SearchRegion(KPlanktonMesh kM, int index, bool type)
    {
        bool[] visited = new bool[kM.Halfedges.Count];

        //create an array to stored visted edge data.
        for (int i = 0; i < kM.Halfedges.Count; i++)
        {
            visited[i] = false;
        }

        //increasement
        if (type)
        {
            //init
            bool escape = false; // nakedStop for reaching mesh border
            List<int> flipStorage = new List<int>();
            List<int> explore = new List<int>();
            visited[index] = true;
            visited[kM.Halfedges.GetPairHalfedge(index)] = true; //this line may got error when input index is on mesh boundary.
            List<int> borders = new List<int>();

            int uNext = kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[kM.Halfedges.GetPairHalfedge(index)].PrevHalfedge)].NextHalfedge;
            int vNext = kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[kM.Halfedges.GetPairHalfedge(index)].NextHalfedge)].PrevHalfedge;
            flipStorage.Add(index); //flipStorage.Add(uNext); flipStorage.Add(vNext);
            explore.Add(index); //explore.Add(uNext); explore.Add(vNext);

            do
            {
                int silent = 0;
                List<int> neighbors = new List<int>();
                foreach (int i in explore) //looping neighbors
                {
                    uNext = kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)].PrevHalfedge)].NextHalfedge;
                    vNext = kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)].NextHalfedge)].PrevHalfedge;
                    bool uIsBorder = kM.Halfedges.IsBoundary(kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)].PrevHalfedge);
                    bool vIsBorder = kM.Halfedges.IsBoundary(kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)].NextHalfedge);
                    // U_dir available
                    if (!kM.Halfedges.IsBoundary(uNext))
                    {
                        for (int k = 0; k < visited.Length; k++) //check visited
                        {
                            if (!visited[uNext]) //not visit before
                            {
                                neighbors.Add(uNext);
                                visited[uNext] = true;
                                visited[kM.Halfedges.GetPairHalfedge(uNext)] = true;
                                flipStorage.Add(uNext);
                            }
                        }
                    }
                    // U_dir on mesh border
                    else
                    {
                        silent++;
                        if (!uIsBorder && !visited[uNext])
                        {
                            borders.Add(uNext);
                            visited[uNext] = true;
                            visited[kM.Halfedges.GetPairHalfedge(uNext)] = true;
                        }
                    }

                    // V_dir available
                    if (!kM.Halfedges.IsBoundary(vNext))
                    {
                        for (int k = 0; k < visited.Length; k++) //check visited
                        {
                            if (!visited[vNext]) //not visit before
                            {
                                neighbors.Add(vNext);
                                visited[vNext] = true;
                                visited[kM.Halfedges.GetPairHalfedge(vNext)] = true;
                                flipStorage.Add(vNext);
                            }
                        }
                    }
                    // V_dir on mesh border
                    else
                    {
                        silent++;
                        if (!vIsBorder && !visited[vNext]) //not visit before
                        {
                            borders.Add(vNext);
                            visited[vNext] = true;
                            visited[kM.Halfedges.GetPairHalfedge(vNext)] = true;
                        }
                    }

                    if (neighbors.Count == 0)
                        escape = true;
                }
                if (!escape)
                    explore = neighbors;
            } while (!escape);

            foreach (int e in flipStorage)
            {
                kM.Halfedges.FlipEdge(e);
            }
            foreach (int e in borders)
            {
                kM.Halfedges.TriangleSplitEdge(e);
            }
        }
        //decreseament
        else
        {
            //init
            bool escape = false; // nakedStop for reaching mesh border
            List<int> flipStorage = new List<int>();
            List<int> explore = new List<int>();
            visited[index] = true;
            visited[kM.Halfedges.GetPairHalfedge(index)] = true; //this line may got error when input index is on mesh boundary.
            List<int> borders = new List<int>();

            int uNext = kM.Halfedges.GetPairHalfedge(kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[index].PrevHalfedge)].NextHalfedge);
            int vNext = kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[kM.Halfedges.GetPairHalfedge(index)].NextHalfedge)].PrevHalfedge;
            flipStorage.Add(index); //flipStorage.Add(uNext); flipStorage.Add(vNext);
            explore.Add(index); //explore.Add(uNext); explore.Add(vNext);

            do
            {
                int silent = 0;
                List<int> neighbors = new List<int>();
                foreach (int i in explore) //looping neighbors
                {
                    uNext = kM.Halfedges.GetPairHalfedge(kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[i].PrevHalfedge)].NextHalfedge);
                    vNext = kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)].NextHalfedge)].PrevHalfedge;
                    bool uIsBorder = kM.Halfedges.IsBoundary(kM.Halfedges[i].PrevHalfedge);
                    bool vIsBorder = kM.Halfedges.IsBoundary(kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)].NextHalfedge);
                    // U_dir available
                    if (!kM.Halfedges.IsBoundary(uNext))
                    {
                        for (int k = 0; k < visited.Length; k++) //check visited
                        {
                            if (!visited[uNext]) //not visit before
                            {
                                neighbors.Add(uNext);
                                visited[uNext] = true;
                                visited[kM.Halfedges.GetPairHalfedge(uNext)] = true;
                                flipStorage.Add(uNext);
                            }
                        }
                    }
                    // U_dir on mesh border
                    else
                    {
                        silent++;
                        if (!uIsBorder && !visited[uNext])
                        {
                            borders.Add(uNext);
                            visited[uNext] = true;
                            visited[kM.Halfedges.GetPairHalfedge(uNext)] = true;
                        }
                    }

                    // V_dir available
                    if (!kM.Halfedges.IsBoundary(vNext))
                    {
                        for (int k = 0; k < visited.Length; k++) //check visited
                        {
                            if (!visited[vNext]) //not visit before
                            {
                                neighbors.Add(vNext);
                                visited[vNext] = true;
                                visited[kM.Halfedges.GetPairHalfedge(vNext)] = true;
                                flipStorage.Add(vNext);
                            }
                        }
                    }

                    // V_dir on mesh border
                    else
                    {
                        silent++;
                        if (!vIsBorder && !visited[vNext]) //not visit before
                        {
                            borders.Add(vNext);
                            visited[vNext] = true;
                            visited[kM.Halfedges.GetPairHalfedge(vNext)] = true;
                        }

                    }
                }
                if (neighbors.Count == 0)
                    escape = true;
                else
                    explore = neighbors;
            } while (!escape);

            foreach (int e in flipStorage)
            {
                kM.Halfedges.FlipEdge(e);
            }
            foreach (int e in borders)
            {
                kM.Halfedges.TriangleSplitEdge(e);
            }
        }
        return true;
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
    // </Custom additional code>  
}
