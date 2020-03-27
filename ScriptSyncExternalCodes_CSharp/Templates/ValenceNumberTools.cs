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

public partial class ValenceNumberTools : GH_ScriptInstance
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


    private void RunScript(Mesh M, DataTree<Point3d> Vertices, DataTree<int> vCount, bool Reset, int subStep, Vector3d shift, ref object Iteration, ref object adM, ref object reM)
    {
        // <Custom code> 
        kM = M.ToKPlanktonMesh();
        //double resetLength = ComputeAverageLength(kM);
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
        
        if (Reset)
        {
            relaxMesh = new MeshRelaxztion(kM);
            relaxMesh.ComputeAverageLength();
            relaxMesh.kM.Vertices.SetVertex(1, kM.Vertices[1].X + 1, kM.Vertices[1].Y + 1, kM.Vertices[1].Z + 1);
            Count = 0;
        }
        else
        {
            if (Count < subStep)
            {
                relaxMesh.Update();
                Count++;
            }
        }
        
        
        adM = kM.ToRhinoMesh();
        Transform move = Transform.Translation(shift);
        Mesh relaxRhMesh = relaxMesh.kM.ToRhinoMesh();
        relaxRhMesh.Transform(move);
        reM = relaxRhMesh;
        Iteration = Count;

        #region component description
        Component.Message = "BurningFront";
        Component.Params.Input[0].Description = "Mesh";
        Component.Params.Input[1].Description = "Singularity point";
        Component.Params.Input[2].Description = "Valence number";
        Component.Params.Input[3].Description = "Reset Relaxation";
        Component.Params.Input[4].Description = "Max Iteration";
        Component.Params.Input[5].Description = "Shift relaxMesh from initMesh";
        Component.Params.Output[1].Description = "Flat State mesh";
        #endregion
        // </Custom code> 
    }

    // <Custom additional code> 
    KPlanktonMesh kM;
    #region Visualize edges
    List<Line> displayEdges = new List<Line>();
    List<int> displayGraphes = new List<int>();
    List<int> displayLabs = new List<int>();

    public override void BeforeRunScript()
    {
        displayGraphes.Clear();
        displayEdges.Clear();
        displayLabs.Clear();
    }

    public void UpdateDisplay(KPlanktonMesh kM)
    {
        foreach(int i in displayGraphes)
        {
            Point3d sDecr = kM.Vertices[kM.Halfedges[i].StartVertex].ToPoint3d();
            Point3d eDecr = kM.Vertices[kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)].StartVertex].ToPoint3d();
            displayEdges.Add(new Line(sDecr, eDecr));

            Point3d sIncr = kM.Vertices[kM.Halfedges[kM.Halfedges[i].PrevHalfedge].StartVertex].ToPoint3d();
            displayEdges.Add(new Line(sIncr, sDecr)); // halfedge data, prevEdge's end point is nextEdge's start point.
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

    int Count = 0;
    MeshRelaxztion relaxMesh;
    //Mesh Relaxzxtion
    public class MeshRelaxztion
    {
        public KPlanktonMesh kM;
        public double AverageLength;
        public double EdgeLengthConstraintWeight = 1;
        public double CollisionWeight = 1;
        public double BendingResistanceWeight = 1;

        private List<Vector3d> totalWeightedMoves;
        private List<double> totalWeights;
        public MeshRelaxztion(KPlanktonMesh initiKMesh)
        {
            kM = initiKMesh;
        }
        public void ComputeAverageLength()
        {
            AverageLength = 0;
            for (int i = 0; i < kM.Halfedges.Count; i += 2)
            {
                Point3d s = kM.Vertices[kM.Halfedges[i].StartVertex].ToPoint3d();
                Line e = new Line(s, kM.Vertices[kM.Halfedges[kM.Halfedges.GetPairHalfedge(i)].StartVertex].ToPoint3d());
                Line ne = new Line(kM.Vertices[kM.Halfedges[kM.Halfedges[i].PrevHalfedge].StartVertex].ToPoint3d(), s);
                AverageLength += e.Length;
            }
            AverageLength = AverageLength / (kM.Halfedges.Count / 2);
        }
        public void Update()
        {
            totalWeightedMoves = new List<Vector3d>();
            totalWeights = new List<double>();

            for (int i = 0; i < kM.Vertices.Count; i++)
            {
                totalWeightedMoves.Add(Vector3d.Zero);
                totalWeights.Add(0.0);
            }
            Spring();
            Collision();
            Bending();

            UpdateVertexPositionsAndVelicities();
        }
        
        private void Spring()
        {
            int halfedgeCount = kM.Halfedges.Count;

            for (int k = 0; k < halfedgeCount; k += 2)
            {
                KPlanktonHalfedge halfedge = kM.Halfedges[k];
                int i = halfedge.StartVertex;
                int j = kM.Halfedges[halfedge.NextHalfedge].StartVertex;

                Vector3d d = kM.Vertices[j].ToPoint3d() - kM.Vertices[i].ToPoint3d();
                double stretchFactor = 1.0 - AverageLength / d.Length;
                if (stretchFactor > 0)
                {
                    Vector3d move = stretchFactor * 0.5 * (d);
                    totalWeightedMoves[i] += move;
                    totalWeightedMoves[j] -= move;
                    totalWeights[i] += EdgeLengthConstraintWeight * 2;
                    totalWeights[j] += EdgeLengthConstraintWeight * 2;
                }
            }
        }
        private void Collision()
        {
            RTree rTree = new RTree();

            for (int i = 0; i < kM.Vertices.Count; i++)
                rTree.Insert(kM.Vertices[i].ToPoint3d(), i);
            List<int>[] collisionIndices = new List<int>[kM.Vertices.Count];

            for (int i = 0; i < kM.Vertices.Count; i++)
                collisionIndices[i] = new List<int>();

            for (int i = 0; i < kM.Vertices.Count; i++)
                rTree.Search(
                    new Sphere(kM.Vertices[i].ToPoint3d(), AverageLength * 2),
                    (sender, args) => { if (i < args.Id) collisionIndices[i].Add(args.Id); });

            for (int i = 0; i < collisionIndices.Length; i++)
            {
                foreach (int j in collisionIndices[i])
                {
                    Vector3d move = kM.Vertices[j].ToPoint3d() - kM.Vertices[i].ToPoint3d();
                    double currentDistance = move.Length;
                    if (currentDistance > AverageLength) continue;
                    move *= 0.5 * (currentDistance - AverageLength) / currentDistance;
                    totalWeightedMoves[i] += CollisionWeight * move;
                    totalWeightedMoves[j] -= CollisionWeight * move;
                    totalWeights[i] += CollisionWeight;
                    totalWeights[j] += CollisionWeight;
                }
            }
        }
        private void Bending()
        {
            int halfedgeCount = kM.Halfedges.Count;
            for (int k = 0; k < halfedgeCount; k += 2)
            {
                // Skip if this edge is naked
                if (kM.Halfedges[k].AdjacentFace == -1 || kM.Halfedges[k + 1].AdjacentFace == -1) continue;

                int i = kM.Halfedges[k].StartVertex;
                int j = kM.Halfedges[k + 1].StartVertex;
                int p = kM.Halfedges[kM.Halfedges[k].PrevHalfedge].StartVertex;
                int q = kM.Halfedges[kM.Halfedges[k + 1].PrevHalfedge].StartVertex;

                Point3d vI = kM.Vertices[i].ToPoint3d();
                Point3d vJ = kM.Vertices[j].ToPoint3d();
                Point3d vP = kM.Vertices[p].ToPoint3d();
                Point3d vQ = kM.Vertices[q].ToPoint3d();

                Vector3d nP = Vector3d.CrossProduct(vJ - vI, vP - vI);
                Vector3d nQ = Vector3d.CrossProduct(vQ - vI, vJ - vI);

                Vector3d planeNormal = (nP + nQ);
                planeNormal.Unitize();

                Point3d planeOrigin = 0.25 * (vI + vJ + vP + vQ);
                Plane plane = new Plane(planeOrigin, planeNormal);
                totalWeightedMoves[i] += BendingResistanceWeight * (plane.ClosestPoint(vI) - vI);
                totalWeightedMoves[j] += BendingResistanceWeight * (plane.ClosestPoint(vJ) - vJ);
                totalWeightedMoves[p] += BendingResistanceWeight * (plane.ClosestPoint(vP) - vP);
                totalWeightedMoves[q] += BendingResistanceWeight * (plane.ClosestPoint(vQ) - vQ);
                totalWeights[i] += BendingResistanceWeight;
                totalWeights[j] += BendingResistanceWeight;
                totalWeights[p] += BendingResistanceWeight;
                totalWeights[q] += BendingResistanceWeight;
            }
        }
        private void UpdateVertexPositionsAndVelicities()
        {
            for (int i = 0; i < kM.Vertices.Count; i++)
            {
                if (totalWeights[i] == 0) continue;
                KPlanktonVertex vertex = kM.Vertices[i];
                Vector3d move = totalWeightedMoves[i] / totalWeights[i];
                kM.Vertices.SetVertex(i, vertex.X + move.X, vertex.Y + move.Y, vertex.Z + move.Z);
            }
        }
    }
    
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
            Point3d sDecr = kM.Vertices[kM.Halfedges[halfs[i]].StartVertex].ToPoint3d();
            Point3d eDecr = kM.Vertices[kM.Halfedges[kM.Halfedges.GetPairHalfedge(halfs[i])].StartVertex].ToPoint3d();
            displayGraphes.Add(halfs[i]);
            displayEdges.Add(new Line(sDecr, eDecr));
            displayLabs.Add(-i - 1);
            
            Point3d sIncr = kM.Vertices[kM.Halfedges[kM.Halfedges[halfs[i]].PrevHalfedge].StartVertex].ToPoint3d();
            displayEdges.Add(new Line(sIncr, sDecr)); // halfedge data, prevEdge's end point is nextEdge's start point.
            displayGraphes.Add(kM.Halfedges[halfs[i]].PrevHalfedge);
            displayLabs.Add(i + 1);
            foreach(int k in E)
            {
                if(k == (-i - 1))
                    Edges.Add(halfs[i]);
                if(k == (i + 1))
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
