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

public partial class ValenceGridTools : GH_ScriptInstance
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
                //Print(type.ToString());
                GridRegion(kM, flipEdge, type);
            }
        }

        //Output
        adM = kM.ToRhinoMesh();

        #region component description
        Component.Message = "FlipRegion";
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
        [increasing]    [decreasing]

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

    public bool GridRegion(KPlanktonMesh kM, int index, bool type)
    {
        //TODO:
        //define grid of flip index and stored visited index, grid size is sqrt clothedges
        int gridSize = 0;
        int[,] grid;
        bool[] visited = new bool[kM.Halfedges.Count];
        
        //create an array to stored visted edge data.
        for(int i = 0; i < kM.Halfedges.Count; i++)
        {
            visited[i] = false;
            if (!kM.Halfedges.IsBoundary(i))
                gridSize++;
        }
        gridSize = (int)Math.Ceiling(Math.Sqrt(gridSize * 0.5));
        grid = new int[gridSize, gridSize];
        //increament
        if (type) 
        {
            int u = 0;
            int v = 0;
            int uNext, vNext = 0; //positive = u direction , negative = v direction
            bool nakedStop; // nakedStop for reaching mesh border

            visited[index] = true;
            visited[kM.Halfedges.GetPairHalfedge(index)] = true; //this line may got error when input index is on mesh boundary.
            grid[u, v] = index; // add first index into array
            do
            {
                uNext = kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[kM.Halfedges.GetPairHalfedge(index)].PrevHalfedge)].NextHalfedge;

                //mesh border detection, test the next is naked edge or adjeceface is on border.
                if (!kM.Halfedges.IsBoundary(uNext)) // (!kM.Halfedges.IsBoundary(uNext) || !kM.Halfedges.IsBoundary(vNext))
                {
                    //start from 00, 01, 02.....
                    // v-loop
                    do
                    {
                        vNext = kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[kM.Halfedges.GetPairHalfedge(index)].NextHalfedge)].PrevHalfedge;
                        if (!kM.Halfedges.IsBoundary(vNext))
                        {
                            index = vNext;
                            visited[vNext] = true;
                            visited[kM.Halfedges.GetPairHalfedge(vNext)] = true;
                            v++;
                            grid[u, v] = vNext;
                            nakedStop = false;
                            //Print("(" + u.ToString() + "," + v.ToString() + "))");
                        }

                        else if (!kM.Halfedges.IsBoundary(kM.Halfedges[kM.Halfedges.GetPairHalfedge(vNext)].PrevHalfedge)
                            || !kM.Halfedges.IsBoundary(kM.Halfedges[kM.Halfedges.GetPairHalfedge(vNext)].NextHalfedge))
                        {
                            nakedStop = true;
                        }

                        else
                        {
                            kM.Halfedges.TriangleSplitEdge(vNext);
                            nakedStop = true;
                        }
                    } while (!nakedStop);
                    v = 0;
                    //end of v-loop
                    
                    
                    index = uNext;
                    visited[uNext] = true;
                    visited[kM.Halfedges.GetPairHalfedge(uNext)] = true;
                    u++;
                    grid[u, v] = uNext;
                    nakedStop = false;
                    //Print("(" + u.ToString() + "," + v.ToString() + ")");
                }

                else if (!kM.Halfedges.IsBoundary(kM.Halfedges[kM.Halfedges.GetPairHalfedge(uNext)].PrevHalfedge)
                    || !kM.Halfedges.IsBoundary(kM.Halfedges[kM.Halfedges.GetPairHalfedge(uNext)].NextHalfedge))
                {
                    nakedStop = true;
                }

                else
                {
                    kM.Halfedges.TriangleSplitEdge(uNext);
                    nakedStop = true;
                }
            } while (!nakedStop);
        }

        //decreasement
        else
        {
            int u = 0;
            int v = 0;
            int uNext, vNext = 0; //positive = u direction , negative = v direction
            bool nakedStop; // nakedStop for reaching mesh border

            visited[index] = true;
            visited[kM.Halfedges.GetPairHalfedge(index)] = true; //this line may got error when input index is on mesh boundary.
            grid[u, v] = index; // add first index into array
            do
            {
                uNext = kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[index].PrevHalfedge)].NextHalfedge;

                //mesh border detection, test the next is naked edge or adjeceface is on border.
                if (!kM.Halfedges.IsBoundary(uNext)) // (!kM.Halfedges.IsBoundary(uNext) || !kM.Halfedges.IsBoundary(vNext))
                {
                    //start from 00, 01, 02.....
                    // v-loop
                    do
                    {
                        vNext = kM.Halfedges[kM.Halfedges.GetPairHalfedge(kM.Halfedges[kM.Halfedges.GetPairHalfedge(index)].NextHalfedge)].PrevHalfedge;
                        if (!kM.Halfedges.IsBoundary(vNext))
                        {
                            index = vNext;
                            visited[vNext] = true;
                            visited[kM.Halfedges.GetPairHalfedge(vNext)] = true;
                            v++;
                            grid[u, v] = vNext;
                            nakedStop = false;
                            Print("(" + u.ToString() + "," + v.ToString() + "))");
                        }

                        else if (!kM.Halfedges.IsBoundary(kM.Halfedges[kM.Halfedges.GetPairHalfedge(vNext)].PrevHalfedge)
                            || !kM.Halfedges.IsBoundary(kM.Halfedges[kM.Halfedges.GetPairHalfedge(vNext)].NextHalfedge))
                        {
                            nakedStop = true;
                        }

                        else
                        {
                            kM.Halfedges.TriangleSplitEdge(vNext);
                            nakedStop = true;
                        }
                    } while (!nakedStop);
                    v = 0;
                    //end of v-loop

                    index = kM.Halfedges.GetPairHalfedge(uNext);
                    visited[uNext] = true;
                    visited[kM.Halfedges.GetPairHalfedge(uNext)] = true;
                    u++;
                    grid[u, v] = uNext;
                    nakedStop = false;
                    Print("(" + u.ToString() + "," + v.ToString() + ")");
                }

                else if (!kM.Halfedges.IsBoundary(kM.Halfedges[kM.Halfedges.GetPairHalfedge(uNext)].PrevHalfedge)
                    || !kM.Halfedges.IsBoundary(kM.Halfedges[kM.Halfedges.GetPairHalfedge(uNext)].NextHalfedge))
                {
                    nakedStop = true;
                }
                
                else
                {
                    //kM.Halfedges.TriangleSplitEdge(uNext);
                    nakedStop = true;
                }
            } while (!nakedStop);
        }

        //flip edges from stored.
        for(int i = 0; i < grid.GetUpperBound(0); i++)
        {
            for(int j = 0; j < grid.GetUpperBound(1); j++)
            {
                kM.Halfedges.FlipEdge(grid[i, j]);
            }
        }
        return true;
    }
    // </Custom additional code>  
}
