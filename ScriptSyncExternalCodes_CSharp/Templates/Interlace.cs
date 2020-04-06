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


    private void RunScript(Mesh M, double S, ref object adM, ref object L, ref object N)
    {
        // <Custom code> 
        List<Polyline> curves = new List<Polyline>();
        List<int>[] nodeCurveID;
        adM = ToInterlaceMesh(M, S, out curves, out nodeCurveID);
        L = curves;
        
        #region Description
        Component.Params.Input[0].Description = "Mesh";
        Component.Params.Input[1].Description = "Amplitude";
        #endregion
        // </Custom code> 
    }

    // <Custom additional code> 
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
        for(int i = 0; i < jointCurves.Length; i ++)
        {
            Polyline pCurve;
            jointCurves[i].TryGetPolyline(out pCurve);
            WeaveCurves.Add(pCurve);
            List<int> ids = new List<int>();
            for (int j = 0; j < pCurve.Count; j++)
            {
                double distance = 10000.0;
                int closestID = 0;
                for(int k = 0; k < jointCurves.Length; k ++)
                {
                    if (k != i)
                    {
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
        }


        return wM;
    }
    // </Custom additional code>  
}
