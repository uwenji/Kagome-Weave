﻿using Rhino;
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


    private void RunScript(Mesh M, DataTree<object> Vertices, DataTree<int> vCount, bool Reset, bool On, object Grab, ref object adM, ref object I)
    {
        // <Custom code> 
        #region Kangaroo
        if (Reset || GoalList == null)
        {
            //init setup values
            displayLabs = new List<int>();
            displayGraphes = new List<int>();
            displayEdges = new List<Line>();
            GoalList = new List<IGoal>();
            PS = new PhysicalSystem();
            double length = 0.0; //for relaxe mesh

            kM = M.ToKPlanktonMesh();
            // set general mesh length
            foreach (KPlanktonHalfedge e in kM.Halfedges)
                length += kM.Vertices[e.StartVertex].ToPoint3d().DistanceTo(kM.Vertices[kM.Halfedges[e.NextHalfedge].StartVertex].ToPoint3d());

            length = length / (kM.Halfedges.Count);

            //prestored vertices index
            storedID = new List<int>(); //Singularity id
            double t = 0.0001; //tolerance

            for (int i = 0; i < Vertices.BranchCount; i++)
            {
                if (Vertices.Branches[i][0] is Rhino.Geometry.Point3d)
                {
                    for (int j = 0; j < kM.Vertices.Count; j++)
                    {
                        Point3d v = ((Rhino.Geometry.Point3d)Vertices.Branches[i][0]);
                        if (kM.Vertices[j].ToPoint3d().DistanceTo(v) <= t)
                            storedID.Add(j);
                    }
                }
                else
                    storedID.Add(Convert.ToInt32(Vertices.Branches[i][0]));
            }
            //region define
            for (int i = 0; i < storedID.Count; i++)
            {
                List<int> flipEdges = InputValence(kM, storedID[i], vCount.Branches[i]);
                for (int j = 0; j < vCount.Branches[i].Count; j++)
                {
                    bool type;
                    if (vCount.Branches[i][j] < 0)
                        type = false;
                    else
                        type = true;
                    SearchRegion(kM, flipEdges[j], type);
                }
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

            if (Grab != null)
                GoalList.Add((IGoal)Grab);
            GoalList.Add(collision);
            UpdateDisplayEdges();
        }
        if(On)
        {
            PS.Step(GoalList, true, 0.001);
            
            for (int i = 0; i < kM.Vertices.Count; i++)
            {
                Point3d kPt = PS.GetPosition(i);
                kM.Vertices[i].X = (float)kPt.X;
                kM.Vertices[i].Y = (float)kPt.Y;
                kM.Vertices[i].Z = (float)kPt.Z;
            }
            UpdateDisplayEdges();
        }
        #endregion

        //output section
        
        Mesh rM = kM.ToRhinoMesh();
        adM = rM;
        I = PS.GetIterations();
        #region component description
        Component.Message = "ValenceTool";
        Component.Params.Input[0].Description = "Mesh";
        Component.Params.Input[1].Description = "Singularity point";
        Component.Params.Input[2].Description = "Valence number";
        Component.Params.Output[1].Description = "Iteration count";
        Component.Params.Output[2].Description = "Adjustment Mesh";
        #endregion
        // </Custom code> 
    }

    // <Custom additional code> 
    KPlanktonMesh kM;
    KangarooSolver.PhysicalSystem PS = new KangarooSolver.PhysicalSystem();
    List<IGoal> GoalList = new List<IGoal>();

    #region Visualize edges
    List<Line> displayEdges = new List<Line>();
    List<int> displayGraphes = new List<int>();
    List<int> displayLabs = new List<int>();
    List<int> storedID = new List<int>();
    public void UpdateDisplayEdges()
    {
        displayEdges = new List<Line>();

        for (int i = 0; i < displayGraphes.Count; i+= 3)
        {
            Point3d sDecr = kM.Vertices[displayGraphes[i]].ToPoint3d();
            Point3d eDecr = kM.Vertices[displayGraphes[i + 1]].ToPoint3d();
            Point3d sIncr = kM.Vertices[displayGraphes[i + 2]].ToPoint3d();

            displayEdges.Add(new Line(sDecr, eDecr));
            displayEdges.Add(new Line(sIncr, sDecr));//halfedge data previous edge's end is current edge's start
        }
    }
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
