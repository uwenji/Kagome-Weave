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


    private void RunScript(Mesh M, DataTree<Point3d> Vertices, DataTree<int> vCount, bool Reset, bool On, ref object adM, ref object A)
    {
        // <Custom code> 
        kM = M.ToKPlanktonMesh();
        //prestored vertices index
        List<int> storedID = new List<int>(); //Singularity id
        double t = 0.0001; //tolerance
        for (int i = 0; i < Vertices.BranchCount; i++)
        {
            for (int j = 0; j < kM.Vertices.Count; j++)
            {
                if (kM.Vertices[j].ToPoint3d().DistanceTo(Vertices.Branches[i][0]) <= t)
                {
                    storedID.Add(j);
                }
            }
        }

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

        Mesh rM = kM.ToRhinoMesh();
        adM = rM;
        #region Kangaroo
        //hinge
        List<Point3d>[] hingePs = KangarooSolver.Util.GetHingePoints(rM);
        for (int i = 0; i < hingePs[0].Count; i++)
        {
            KangarooSolver.IGoal hinge = new KangarooSolver.Goals.Hinge(hingePs[0][i], hingePs[1][i], hingePs[2][i], hingePs[3][i], 0.0, 1.0);
            //GoalList.Add(hinge);
        }
        //spring
        List<Line> edges = new List<Line>();
        List<Line> nextEdges = new List<Line>();
        List<Point3d> vertices = new List<Point3d>();
        double averageLength = 0;
        for (int i = 0; i < kM.Halfedges.Count; i += 2)
        {
            Point3d s = kM.Vertices[kM.Halfedges[i].StartVertex].ToPoint3d();
            Line e = new Line(s, kM.Vertices[kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)].StartVertex].ToPoint3d());
            Line ne = new Line(kM.Vertices[kM.Halfedges[kM.Halfedges[i].PrevHalfedge].StartVertex].ToPoint3d(), s);
            edges.Add(e); nextEdges.Add(ne);
            vertices.Add(s);
            averageLength += e.Length;
        }
        averageLength = averageLength / (kM.Halfedges.Count / 2);
        foreach (Line l in edges)
        {
            KangarooSolver.IGoal sping = new KangarooSolver.Goals.Spring(l, averageLength, 1.0);
            GoalList.Add(sping);
        }
        //collision
        KangarooSolver.IGoal sphCollision = new KangarooSolver.Goals.SphereCollide(vertices, averageLength, 1.0);
        //GoalList.Add(sphCollision);
        //equalAngle
        KangarooSolver.IGoal equalAngle = new KangarooSolver.Goals.EqualAngle(edges, nextEdges, 1.0);
        //GoalList.Add(equalAngle);

        if (Reset)
        {
            displayLabs = new List<int>();
            displayGraphes = new List<int>();
            displayEdges = new List<Line>();
            
            PS = new KangarooSolver.PhysicalSystem();
            GoalList = new List<IGoal>();
            foreach (IGoal G in GoalList) //Assign indexes to the particles in each Goal:
            {
                PS.AssignPIndex(G, 0.01); // the second argument is the tolerance distance below which points combine into a single particle
            }
        }
        else
        {
            UpdateDisplayEdges();
        }
        #endregion
        A = displayGraphes;
        /*
        if (subStep)
        {
          PS.Step(GoalList, true, 0.001);
          counter++;
        }*/

        #region component description
        Component.Message = "BurningFront";
        Component.Params.Input[0].Description = "Mesh";
        Component.Params.Input[1].Description = "Singularity point";
        Component.Params.Input[2].Description = "Valence number";
        Component.Params.Output[1].Description = "Flat State mesh";
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
    // </Custom additional code>  
}
