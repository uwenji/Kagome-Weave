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

#region padding (this ensures the line number of this file match with those in the code editor of the C# Script component











#endregion

public partial class ValenceTools : GH_ScriptInstance
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


    private void RunScript(Mesh M, DataTree<Point3d> Vertices, DataTree<Line> Edges, ref object adM, ref object L, ref object N)
    {
        // <Custom code> 
        KPlanktonMesh kM = M.ToKPlanktonMesh();
        for (int i = 0; i < Vertices.BranchCount; i++)
        {
            for (int j = 0; j < Edges.Branches[i].Count; j++)
            {
                bool type;
                int flipEdge = DefineType(kM, Vertices.Branches[i][0], Edges.Branches[i][j], out type);
                Print(type.ToString());
                SearchRegion(kM, flipEdge, type);
            }
        }

        //Output
        adM = kM.ToRhinoMesh();

        #region component description
        Component.Message = "BurningFront";
        Component.Params.Input[0].Description = "Mesh";
        Component.Params.Input[1].Description = "Singularity point, multiple vertices required data structure";
        Component.Params.Input[2].Description = "Flip edge, multiple edges required data structure";
        #endregion
        // </Custom code> 
    }

    // <Custom additional code> 

    public int DefineType(KPlanktonMesh kM, Point3d V, Line E, out bool type)
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
        //valence decreasing
        double t = 0.0001; //tolerance
        int index = -1;
        type = false;
        if (V.DistanceTo(E.From) < t || V.DistanceTo(E.To) < t)
        {
            for (int i = 0; i < kM.Vertices.Count; i++)
            {
                KPlanktonVertex vertex = kM.Vertices[i];
                if (V.DistanceTo(vertex.ToPoint3d()) < t)
                {
                    Print(i.ToString());
                    int[] edges = kM.Vertices.GetIncomingHalfedges(i);
                    foreach (int incomingE in edges)
                    {
                        Point3d edgeStart = kM.Vertices[kM.Halfedges[incomingE].StartVertex].ToPoint3d();
                        Point3d edgeEnd = kM.Vertices[kM.Halfedges[kM.Halfedges[incomingE].NextHalfedge].StartVertex].ToPoint3d();
                        if ((E.From.DistanceTo(edgeStart) < t && E.To.DistanceTo(edgeEnd) < t) || (E.From.DistanceTo(edgeEnd) < t && E.To.DistanceTo(edgeStart) < t))
                        {
                            index = incomingE;
                            type = false;
                            break;
                        }

                    }
                }
            }
        }

        //valence increasing
        else
        {
            //loccation vertex id from plankton then find which halfedge match given line
            for (int i = 0; i < kM.Vertices.Count; i++)
            {
                KPlanktonVertex vertex = kM.Vertices[i];
                if (V.DistanceTo(vertex.ToPoint3d()) < t)
                {
                    int[] edges = kM.Vertices.GetIncomingHalfedges(i);
                    foreach (int incomingE in edges)
                    {
                        int flipID = kM.Halfedges[incomingE].PrevHalfedge;
                        Point3d edgeStart = kM.Vertices[kM.Halfedges[flipID].StartVertex].ToPoint3d();
                        Point3d edgeEnd = kM.Vertices[kM.Halfedges[kM.Halfedges.GetPairHalfedge(flipID)].StartVertex].ToPoint3d();
                        if ((E.From.DistanceTo(edgeStart) < t && E.To.DistanceTo(edgeEnd) < t) || (E.From.DistanceTo(edgeEnd) < t && E.To.DistanceTo(edgeStart) < t))
                        {
                            index = flipID;
                            type = true;
                            break;
                        }
                    }
                }
            }
        }
        return index;
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

            foreach(int e in flipStorage)
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
