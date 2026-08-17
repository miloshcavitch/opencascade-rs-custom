#include "rust/cxx.h"
#include <BOPAlgo_GlueEnum.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeShapeOnMesh.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFeat_MakeCylindricalHole.hxx>
#include <BRepFeat_MakeDPrism.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeFillet2d.hxx>
#include <BRepGProp.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <BRepLib.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeAnalysis.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <GProp_GProps.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_JoinType.hxx>
#include <GeomConvert.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <IGESControl_Reader.hxx>
#include <IGESControl_Writer.hxx>
#include <IMeshData_Status.hxx>
#include <IMeshTools_MeshAlgoType.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Law_Function.hxx>
#include <Law_Interpol.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <Poly_Connect.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Type.hxx>
#include <StlAPI_Writer.hxx>
#include <TColgp_Array1OfDir.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

// Generic template constructor
template <typename T, typename... Args> std::unique_ptr<T> construct_unique(Args... args) {
  return std::unique_ptr<T>(new T(args...));
}

// Generic List
template <typename T> std::unique_ptr<std::vector<T>> list_to_vector(const NCollection_List<T> &list) {
  return std::unique_ptr<std::vector<T>>(new std::vector<T>(list.begin(), list.end()));
}

// Handles
typedef opencascade::handle<Standard_Type> HandleStandardType;
typedef opencascade::handle<Geom_Curve> HandleGeomCurve;
typedef opencascade::handle<Geom_BSplineCurve> HandleGeomBSplineCurve;
typedef opencascade::handle<Geom_BezierCurve> HandleGeomBezierCurve;
typedef opencascade::handle<Geom_TrimmedCurve> HandleGeomTrimmedCurve;
typedef opencascade::handle<Geom_Surface> HandleGeomSurface;
typedef opencascade::handle<Geom_BezierSurface> HandleGeomBezierSurface;
typedef opencascade::handle<Geom_Plane> HandleGeomPlane;
typedef opencascade::handle<Geom2d_Curve> HandleGeom2d_Curve;
typedef opencascade::handle<Geom2d_Ellipse> HandleGeom2d_Ellipse;
typedef opencascade::handle<Geom2d_TrimmedCurve> HandleGeom2d_TrimmedCurve;
typedef opencascade::handle<Geom_CylindricalSurface> HandleGeom_CylindricalSurface;
typedef opencascade::handle<Poly_Triangulation> HandlePoly_Triangulation;
typedef opencascade::handle<TopTools_HSequenceOfShape> HandleTopTools_HSequenceOfShape;
typedef opencascade::handle<Law_Function> HandleLawFunction;

typedef opencascade::handle<TColgp_HArray1OfPnt> Handle_TColgpHArray1OfPnt;

inline std::unique_ptr<Handle_TColgpHArray1OfPnt>
new_HandleTColgpHArray1OfPnt_from_TColgpHArray1OfPnt(std::unique_ptr<TColgp_HArray1OfPnt> array) {
  return std::unique_ptr<Handle_TColgpHArray1OfPnt>(new Handle_TColgpHArray1OfPnt(array.release()));
}

// Handle stuff
template <typename T> const T &handle_try_deref(const opencascade::handle<T> &handle) {
  if (handle.IsNull()) {
    throw std::runtime_error("null handle dereference");
  }
  return *handle;
}

inline const HandleStandardType &DynamicType(const HandleGeomSurface &surface) { return surface->DynamicType(); }

inline rust::String type_name(const HandleStandardType &handle) { return std::string(handle->Name()); }

inline std::unique_ptr<gp_Pnt> HandleGeomCurve_Value(const HandleGeomCurve &curve, const Standard_Real U) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(curve->Value(U)));
}

inline std::unique_ptr<gp_Pnt> GCPnts_TangentialDeflection_Value(const GCPnts_TangentialDeflection &approximator,
                                                                 Standard_Integer i) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(approximator.Value(i)));
}

inline std::unique_ptr<HandleGeomPlane> new_HandleGeomPlane_from_HandleGeomSurface(const HandleGeomSurface &surface) {
  HandleGeomPlane plane_handle = opencascade::handle<Geom_Plane>::DownCast(surface);
  return std::unique_ptr<HandleGeomPlane>(new opencascade::handle<Geom_Plane>(plane_handle));
}

// Collections
inline void shape_list_append_face(TopTools_ListOfShape &list, const TopoDS_Face &face) { list.Append(face); }

// Geometry
inline const gp_Pnt &handle_geom_plane_location(const HandleGeomPlane &plane) { return plane->Location(); }

inline std::unique_ptr<HandleGeom_CylindricalSurface> Geom_CylindricalSurface_ctor(const gp_Ax3 &axis, double radius) {
  return std::unique_ptr<HandleGeom_CylindricalSurface>(
      new opencascade::handle<Geom_CylindricalSurface>(new Geom_CylindricalSurface(axis, radius)));
}

inline std::unique_ptr<HandleGeomBSplineCurve> GeomAPI_Interpolate_Curve(const GeomAPI_Interpolate &interpolate) {
  return std::unique_ptr<HandleGeomBSplineCurve>(new opencascade::handle<Geom_BSplineCurve>(interpolate.Curve()));
}

inline std::unique_ptr<HandleGeomBezierCurve>
Geom_BezierCurve_to_handle(std::unique_ptr<Geom_BezierCurve> bezier_curve) {
  return std::unique_ptr<HandleGeomBezierCurve>(new HandleGeomBezierCurve(bezier_curve.release()));
}

inline std::unique_ptr<HandleGeomSurface> cylinder_to_surface(const HandleGeom_CylindricalSurface &cylinder_handle) {
  return std::unique_ptr<HandleGeomSurface>(new opencascade::handle<Geom_Surface>(cylinder_handle));
}

inline std::unique_ptr<HandleGeomBezierSurface> Geom_BezierSurface_ctor(const TColgp_Array2OfPnt &poles) {
  return std::unique_ptr<HandleGeomBezierSurface>(
      new opencascade::handle<Geom_BezierSurface>(new Geom_BezierSurface(poles)));
}

inline std::unique_ptr<HandleGeomSurface> bezier_to_surface(const HandleGeomBezierSurface &bezier_handle) {
  return std::unique_ptr<HandleGeomSurface>(new opencascade::handle<Geom_Surface>(bezier_handle));
}

inline std::unique_ptr<HandleGeom2d_Ellipse> Geom2d_Ellipse_ctor(const gp_Ax2d &axis, double major_radius,
                                                                 double minor_radius) {
  return std::unique_ptr<HandleGeom2d_Ellipse>(
      new opencascade::handle<Geom2d_Ellipse>(new Geom2d_Ellipse(axis, major_radius, minor_radius)));
}

inline std::unique_ptr<HandleGeom2d_Curve> ellipse_to_HandleGeom2d_Curve(const HandleGeom2d_Ellipse &ellipse_handle) {
  return std::unique_ptr<HandleGeom2d_Curve>(new opencascade::handle<Geom2d_Curve>(ellipse_handle));
}

inline std::unique_ptr<HandleGeom2d_TrimmedCurve> Geom2d_TrimmedCurve_ctor(const HandleGeom2d_Curve &curve, double u1,
                                                                           double u2) {
  return std::unique_ptr<HandleGeom2d_TrimmedCurve>(
      new opencascade::handle<Geom2d_TrimmedCurve>(new Geom2d_TrimmedCurve(curve, u1, u2)));
}

inline std::unique_ptr<HandleGeom2d_Curve>
HandleGeom2d_TrimmedCurve_to_curve(const HandleGeom2d_TrimmedCurve &trimmed_curve) {
  return std::unique_ptr<HandleGeom2d_Curve>(new opencascade::handle<Geom2d_Curve>(trimmed_curve));
}

inline std::unique_ptr<gp_Pnt2d> ellipse_value(const HandleGeom2d_Ellipse &ellipse, double u) {
  return std::unique_ptr<gp_Pnt2d>(new gp_Pnt2d(ellipse->Value(u)));
}

// Segment Stuff
inline std::unique_ptr<HandleGeomTrimmedCurve> GC_MakeSegment_Value(const GC_MakeSegment &segment) {
  return std::unique_ptr<HandleGeomTrimmedCurve>(new opencascade::handle<Geom_TrimmedCurve>(segment.Value()));
}

inline std::unique_ptr<HandleGeom2d_TrimmedCurve> GCE2d_MakeSegment_point_point(const gp_Pnt2d &p1,
                                                                                const gp_Pnt2d &p2) {
  return std::unique_ptr<HandleGeom2d_TrimmedCurve>(
      new opencascade::handle<Geom2d_TrimmedCurve>(GCE2d_MakeSegment(p1, p2)));
}

// Arc stuff
inline std::unique_ptr<HandleGeomTrimmedCurve> GC_MakeArcOfCircle_Value(const GC_MakeArcOfCircle &arc) {
  return std::unique_ptr<HandleGeomTrimmedCurve>(new opencascade::handle<Geom_TrimmedCurve>(arc.Value()));
}

inline std::unique_ptr<gp_Pnt> BRepAdaptor_Curve_value(const BRepAdaptor_Curve &curve, const Standard_Real U) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(curve.Value(U)));
}

// BRepLib
inline bool BRepLibBuildCurves3d(const TopoDS_Shape &shape) { return BRepLib::BuildCurves3d(shape); }

inline void MakeThickSolidByJoin(BRepOffsetAPI_MakeThickSolid &make_thick_solid, const TopoDS_Shape &shape,
                                 const TopTools_ListOfShape &closing_faces, const Standard_Real offset,
                                 const Standard_Real tolerance) {
  make_thick_solid.MakeThickSolidByJoin(shape, closing_faces, offset, tolerance);
}

// Geometric processing
inline const gp_Ax1 &gp_OX() { return gp::OX(); }
inline const gp_Ax1 &gp_OY() { return gp::OY(); }
inline const gp_Ax1 &gp_OZ() { return gp::OZ(); }

inline const gp_Dir &gp_DZ() { return gp::DZ(); }

inline std::unique_ptr<gp_Ax1> gp_Ax1_ctor(const gp_Pnt &origin, const gp_Dir &main_dir) {
  return std::unique_ptr<gp_Ax1>(new gp_Ax1(origin, main_dir));
}

inline std::unique_ptr<gp_Ax2> gp_Ax2_ctor(const gp_Pnt &origin, const gp_Dir &main_dir) {
  return std::unique_ptr<gp_Ax2>(new gp_Ax2(origin, main_dir));
}

inline std::unique_ptr<gp_Ax3> gp_Ax3_from_gp_Ax2(const gp_Ax2 &axis) {
  return std::unique_ptr<gp_Ax3>(new gp_Ax3(axis));
}

inline std::unique_ptr<gp_Dir> gp_Dir_ctor(double x, double y, double z) {
  return std::unique_ptr<gp_Dir>(new gp_Dir(x, y, z));
}

inline std::unique_ptr<gp_Dir2d> gp_Dir2d_ctor(double x, double y) {
  return std::unique_ptr<gp_Dir2d>(new gp_Dir2d(x, y));
}

inline std::unique_ptr<gp_Ax2d> gp_Ax2d_ctor(const gp_Pnt2d &point, const gp_Dir2d &dir) {
  return std::unique_ptr<gp_Ax2d>(new gp_Ax2d(point, dir));
}

// Law_Function stuff
inline std::unique_ptr<HandleLawFunction> Law_Function_to_handle(std::unique_ptr<Law_Function> law_function) {
  return std::unique_ptr<HandleLawFunction>(new HandleLawFunction(law_function.release()));
}

// Law_Interpol stuff
inline std::unique_ptr<Law_Function> Law_Interpol_into_Law_Function(std::unique_ptr<Law_Interpol> law_interpol) {
  return std::unique_ptr<Law_Function>(law_interpol.release());
}

// Shape stuff
inline const TopoDS_Vertex &TopoDS_cast_to_vertex(const TopoDS_Shape &shape) { return TopoDS::Vertex(shape); }
inline const TopoDS_Edge &TopoDS_cast_to_edge(const TopoDS_Shape &shape) { return TopoDS::Edge(shape); }
inline const TopoDS_Wire &TopoDS_cast_to_wire(const TopoDS_Shape &shape) { return TopoDS::Wire(shape); }
inline const TopoDS_Face &TopoDS_cast_to_face(const TopoDS_Shape &shape) { return TopoDS::Face(shape); }
inline const TopoDS_Shell &TopoDS_cast_to_shell(const TopoDS_Shape &shape) { return TopoDS::Shell(shape); }
inline const TopoDS_Solid &TopoDS_cast_to_solid(const TopoDS_Shape &shape) { return TopoDS::Solid(shape); }
inline const TopoDS_Compound &TopoDS_cast_to_compound(const TopoDS_Shape &shape) { return TopoDS::Compound(shape); }

inline const TopoDS_Shape &cast_vertex_to_shape(const TopoDS_Vertex &vertex) { return vertex; }
inline const TopoDS_Shape &cast_edge_to_shape(const TopoDS_Edge &edge) { return edge; }
inline const TopoDS_Shape &cast_wire_to_shape(const TopoDS_Wire &wire) { return wire; }
inline const TopoDS_Shape &cast_face_to_shape(const TopoDS_Face &face) { return face; }
inline const TopoDS_Shape &cast_shell_to_shape(const TopoDS_Shell &shell) { return shell; }
inline const TopoDS_Shape &cast_solid_to_shape(const TopoDS_Solid &solid) { return solid; }
inline const TopoDS_Shape &cast_compound_to_shape(const TopoDS_Compound &compound) { return compound; }

// Compound shapes
inline std::unique_ptr<TopoDS_Shape> TopoDS_Compound_as_shape(std::unique_ptr<TopoDS_Compound> compound) {
  return compound;
}

inline std::unique_ptr<TopoDS_Shape> TopoDS_Shell_as_shape(std::unique_ptr<TopoDS_Shell> shell) { return shell; }

inline const TopoDS_Builder &BRep_Builder_upcast_to_topods_builder(const BRep_Builder &builder) { return builder; }

// Transforms
inline std::unique_ptr<HandleGeomSurface> BRep_Tool_Surface(const TopoDS_Face &face) {
  return std::unique_ptr<HandleGeomSurface>(new opencascade::handle<Geom_Surface>(BRep_Tool::Surface(face)));
}

inline std::unique_ptr<HandleGeomCurve> BRep_Tool_Curve(const TopoDS_Edge &edge, Standard_Real &first,
                                                        Standard_Real &last) {
  return std::unique_ptr<HandleGeomCurve>(new opencascade::handle<Geom_Curve>(BRep_Tool::Curve(edge, first, last)));
}

inline std::unique_ptr<gp_Pnt> BRep_Tool_Pnt(const TopoDS_Vertex &vertex) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(BRep_Tool::Pnt(vertex)));
}

inline std::unique_ptr<gp_Trsf> TopLoc_Location_Transformation(const TopLoc_Location &location) {
  return std::unique_ptr<gp_Trsf>(new gp_Trsf(location.Transformation()));
}

// IMeshTools_Parameters — the mesher's full parameter set.
//
// BRepMesh_IncrementalMesh's five-argument constructor reaches only four of these
// fourteen fields (Deflection, Angle, Relative, InParallel) and leaves the rest at
// defaults that are not all inert. Two in particular:
//
//   - AngleInterior and DeflectionInterior default to -1.0, and initParameters()
//     resolves the first to *2.0 * Angle*. So a caller asking for 0.5 rad gets 0.5
//     on the boundary edges and 1.0 across the face interior.
//   - MinSize defaults to 0.1 * min(Deflection, DeflectionInterior), a floor on
//     triangle edge length. It is the kernel's own guard against amplification on
//     distorted surfaces, and the only one that does not scale with the whole shape.
//
// Bound as an opaque type with one setter per field rather than a single positional
// constructor. Nine consecutive Standard_Real / Standard_Boolean arguments transpose
// silently, and the failure would arrive as a subtly wrong mesh rather than as a
// compile error. Field order is not part of this API.
inline void IMeshTools_Parameters_set_deflection(IMeshTools_Parameters &params, const Standard_Real value) {
  params.Deflection = value;
}

inline void IMeshTools_Parameters_set_angle(IMeshTools_Parameters &params, const Standard_Real value) {
  params.Angle = value;
}

// Negative leaves it to initParameters(), which copies Deflection.
inline void IMeshTools_Parameters_set_deflection_interior(IMeshTools_Parameters &params,
                                                          const Standard_Real value) {
  params.DeflectionInterior = value;
}

// Negative leaves it to initParameters(), which uses 2.0 * Angle.
inline void IMeshTools_Parameters_set_angle_interior(IMeshTools_Parameters &params, const Standard_Real value) {
  params.AngleInterior = value;
}

// Negative leaves it to initParameters(), which uses
// IMeshTools_Parameters::RelMinSize() * min(Deflection, DeflectionInterior).
inline void IMeshTools_Parameters_set_min_size(IMeshTools_Parameters &params, const Standard_Real value) {
  params.MinSize = value;
}

inline void IMeshTools_Parameters_set_adjust_min_size(IMeshTools_Parameters &params, const Standard_Boolean value) {
  params.AdjustMinSize = value;
}

inline void IMeshTools_Parameters_set_control_surface_deflection(IMeshTools_Parameters &params,
                                                                 const Standard_Boolean value) {
  params.ControlSurfaceDeflection = value;
}

// Controls the direction of the mesher's reuse test. BRepMesh_Deflection::IsConsistent
// is
//
//   current < (1 + ratio) * required  &&  (!allowDecrease || current > (1 - ratio) * required)
//
// so with this off — OCCT's default — a stored triangulation finer than the request
// counts as consistent and is kept, and a *loosening* request is silently ignored.
// Turning it on makes the test a band in both directions. It does not make an angular
// change observable: that comparison is on linear deflection alone, and
// Poly_Triangulation stores no angle. Dropping a stale triangulation entirely is still
// BRepTools_Clean's job.
inline void IMeshTools_Parameters_set_allow_quality_decrease(IMeshTools_Parameters &params,
                                                             const Standard_Boolean value) {
  params.AllowQualityDecrease = value;
}

// Takes the raw enumerator rather than a bridged enum: IMeshTools_MeshAlgoType is an
// unscoped enum with no fixed underlying type, so its size is implementation-defined
// and a cxx extern-enum binding would rest on a static assert about it. -1 DEFAULT,
// 0 Watson, 1 Delabella; anything else is clamped to DEFAULT rather than cast into an
// enumerator that does not exist.
inline void IMeshTools_Parameters_set_mesh_algo(IMeshTools_Parameters &params, const int value) {
  switch (value) {
    case 0:
      params.MeshAlgo = IMeshTools_MeshAlgoType_Watson;
      break;
    case 1:
      params.MeshAlgo = IMeshTools_MeshAlgoType_Delabella;
      break;
    default:
      params.MeshAlgo = IMeshTools_MeshAlgoType_DEFAULT;
      break;
  }
}

// The three-argument constructor, taking its Message_ProgressRange default. Like every
// other non-default BRepMesh_IncrementalMesh constructor it calls Perform() itself, so
// the parameters must be complete before this is called — nothing set afterwards
// reaches the mesh.
inline std::unique_ptr<BRepMesh_IncrementalMesh>
BRepMesh_IncrementalMesh_ctor_params(const TopoDS_Shape &shape, const IMeshTools_Parameters &params) {
  return std::unique_ptr<BRepMesh_IncrementalMesh>(new BRepMesh_IncrementalMesh(shape, params));
}

inline std::unique_ptr<HandlePoly_Triangulation>
HandlePoly_Triangulation_ctor(std::unique_ptr<Poly_Triangulation> triangulation) {
  return std::unique_ptr<HandlePoly_Triangulation>(new HandlePoly_Triangulation(triangulation.release()));
}

inline std::unique_ptr<HandlePoly_Triangulation> BRep_Tool_Triangulation(const TopoDS_Face &face,
                                                                         TopLoc_Location &location) {
  return std::unique_ptr<HandlePoly_Triangulation>(
      new opencascade::handle<Poly_Triangulation>(BRep_Tool::Triangulation(face, location)));
}

inline std::unique_ptr<TopoDS_Shape> ExplorerCurrentShape(const TopExp_Explorer &explorer) {
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(explorer.Current()));
}

inline std::unique_ptr<TopoDS_Vertex> TopExp_FirstVertex(const TopoDS_Edge &edge) {
  return std::unique_ptr<TopoDS_Vertex>(new TopoDS_Vertex(TopExp::FirstVertex(edge)));
}

inline std::unique_ptr<TopoDS_Vertex> TopExp_LastVertex(const TopoDS_Edge &edge) {
  return std::unique_ptr<TopoDS_Vertex>(new TopoDS_Vertex(TopExp::LastVertex(edge)));
}

inline void TopExp_EdgeVertices(const TopoDS_Edge &edge, TopoDS_Vertex &vertex1, TopoDS_Vertex &vertex2) {
  return TopExp::Vertices(edge, vertex1, vertex2);
}

inline void TopExp_WireVertices(const TopoDS_Wire &wire, TopoDS_Vertex &vertex1, TopoDS_Vertex &vertex2) {
  return TopExp::Vertices(wire, vertex1, vertex2);
}

inline bool TopExp_CommonVertex(const TopoDS_Edge &edge1, const TopoDS_Edge &edge2, TopoDS_Vertex &vertex) {
  return TopExp::CommonVertex(edge1, edge2, vertex);
}

inline std::unique_ptr<TopoDS_Face> BRepIntCurveSurface_Inter_face(const BRepIntCurveSurface_Inter &intersector) {
  return std::unique_ptr<TopoDS_Face>(new TopoDS_Face(intersector.Face()));
}

inline std::unique_ptr<gp_Pnt> BRepIntCurveSurface_Inter_point(const BRepIntCurveSurface_Inter &intersector) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(intersector.Pnt()));
}

// BRepFeat
inline std::unique_ptr<BRepFeat_MakeCylindricalHole> BRepFeat_MakeCylindricalHole_ctor() {
  return std::unique_ptr<BRepFeat_MakeCylindricalHole>(new BRepFeat_MakeCylindricalHole());
}

// Data Import
inline IFSelect_ReturnStatus read_step(STEPControl_Reader &reader, rust::String theFileName) {
  return reader.ReadFile(theFileName.c_str());
}

inline IFSelect_ReturnStatus read_iges(IGESControl_Reader &reader, rust::String theFileName) {
  return reader.ReadFile(theFileName.c_str());
}

inline std::unique_ptr<TopoDS_Shape> one_shape_step(const STEPControl_Reader &reader) {
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(reader.OneShape()));
}

inline std::unique_ptr<TopoDS_Shape> one_shape_iges(const IGESControl_Reader &reader) {
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(reader.OneShape()));
}

// Data Export
inline IFSelect_ReturnStatus transfer_shape(STEPControl_Writer &writer, const TopoDS_Shape &theShape) {
  return writer.Transfer(theShape, STEPControl_AsIs);
}

inline void compute_model(IGESControl_Writer &writer) { writer.ComputeModel(); }

inline bool add_shape(IGESControl_Writer &writer, const TopoDS_Shape &theShape) { return writer.AddShape(theShape); }

inline IFSelect_ReturnStatus write_step(STEPControl_Writer &writer, rust::String theFileName) {
  return writer.Write(theFileName.c_str());
}

inline bool write_iges(IGESControl_Writer &writer, rust::String theFileName) {
  return writer.Write(theFileName.c_str());
}

inline bool write_stl(StlAPI_Writer &writer, const TopoDS_Shape &theShape, rust::String theFileName) {
  return writer.Write(theShape, theFileName.c_str());
}

inline std::unique_ptr<gp_Dir> Poly_Triangulation_Normal(const Poly_Triangulation &triangulation,
                                                         const Standard_Integer index) {
  return std::unique_ptr<gp_Dir>(new gp_Dir(triangulation.Normal(index)));
}

inline std::unique_ptr<gp_Pnt> Poly_Triangulation_Node(const Poly_Triangulation &triangulation,
                                                       const Standard_Integer index) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(triangulation.Node(index)));
}

inline std::unique_ptr<gp_Pnt2d> Poly_Triangulation_UV(const Poly_Triangulation &triangulation,
                                                       const Standard_Integer index) {
  return std::unique_ptr<gp_Pnt2d>(new gp_Pnt2d(triangulation.UVNode(index)));
}

inline void compute_normals(const TopoDS_Face &face, const Handle(Poly_Triangulation) & triangulation) {
  BRepLib_ToolTriangulatedShape::ComputeNormals(face, triangulation);
}

// Shape Properties
inline std::unique_ptr<gp_Pnt> GProp_GProps_CentreOfMass(const GProp_GProps &props) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(props.CentreOfMass()));
}

inline void BRepGProp_LinearProperties(const TopoDS_Shape &shape, GProp_GProps &props) {
  BRepGProp::LinearProperties(shape, props);
}

inline void BRepGProp_SurfaceProperties(const TopoDS_Shape &shape, GProp_GProps &props) {
  BRepGProp::SurfaceProperties(shape, props);
}

inline void BRepGProp_VolumeProperties(const TopoDS_Shape &shape, GProp_GProps &props) {
  BRepGProp::VolumeProperties(shape, props);
}

// Fillets
inline std::unique_ptr<TopoDS_Edge> BRepFilletAPI_MakeFillet2d_add_fillet(BRepFilletAPI_MakeFillet2d &make_fillet,
                                                                          const TopoDS_Vertex &vertex,
                                                                          Standard_Real radius) {
  return std::unique_ptr<TopoDS_Edge>(new TopoDS_Edge(make_fillet.AddFillet(vertex, radius)));
}

// Chamfers
inline std::unique_ptr<TopoDS_Edge>
BRepFilletAPI_MakeFillet2d_add_chamfer(BRepFilletAPI_MakeFillet2d &make_fillet, const TopoDS_Edge &edge1,
                                       const TopoDS_Edge &edge2, const Standard_Real dist1, const Standard_Real dist2) {
  return std::unique_ptr<TopoDS_Edge>(new TopoDS_Edge(make_fillet.AddChamfer(edge1, edge2, dist1, dist2)));
}

inline std::unique_ptr<TopoDS_Edge>
BRepFilletAPI_MakeFillet2d_add_chamfer_angle(BRepFilletAPI_MakeFillet2d &make_fillet, const TopoDS_Edge &edge,
                                             const TopoDS_Vertex &vertex, const Standard_Real dist,
                                             const Standard_Real angle) {
  return std::unique_ptr<TopoDS_Edge>(new TopoDS_Edge(make_fillet.AddChamfer(edge, vertex, dist, angle)));
}

// BRepTools
inline std::unique_ptr<TopoDS_Wire> outer_wire(const TopoDS_Face &face) {
  return std::unique_ptr<TopoDS_Wire>(new TopoDS_Wire(BRepTools::OuterWire(face)));
}

// Drops every triangulation the shape is carrying: the Poly_Triangulation on each
// face, and the Poly_PolygonOnTriangulation / Poly_Polygon3D on each edge. The next
// mesher call then has nothing to reuse and meshes from the surfaces.
//
// This is not a tidy-up, it is what makes a meshing parameter observable.
// BRepMesh_IncrementalMesh is incremental as advertised: it keeps any face whose
// stored triangulation already satisfies the request, and that test
// (BRepMesh_Deflection::IsConsistent) reads the LINEAR deflection alone, comparing
// current < 1.1 * required. So a changed angular deflection can never trigger a
// re-mesh — Poly_Triangulation does not store an angle to compare against — and a
// linear deflection that moved by less than a ninth is ignored too. Without this
// call, tightening a parameter silently returns the old mesh.
//
// Clean takes a const reference and mutates through it. That is OCCT's own
// signature, not a cast here: the triangulation lives in the shared TShape, which
// the handle only points at. It follows that this is a data race if two threads
// hold the same shape — callers must keep one shape on one thread.
inline void BRepTools_Clean(const TopoDS_Shape &shape) {
  try {
    BRepTools::Clean(shape);
  } catch (...) {
    // Nothing to report and nothing to undo: a shape that could not be cleaned
    // still meshes, it just may reuse what it was already carrying. The catch is
    // here so an OCCT exception cannot unwind across the FFI boundary.
  }
}

// Collections
inline void map_shapes(const TopoDS_Shape &S, const TopAbs_ShapeEnum T, TopTools_IndexedMapOfShape &M) {
  TopExp::MapShapes(S, T, M);
}

inline void map_shapes_and_ancestors(const TopoDS_Shape &S, const TopAbs_ShapeEnum TS, const TopAbs_ShapeEnum TA,
                                     TopTools_IndexedDataMapOfShapeListOfShape &M) {
  TopExp::MapShapesAndAncestors(S, TS, TA, M);
}

inline void map_shapes_and_unique_ancestors(const TopoDS_Shape &S, const TopAbs_ShapeEnum TS, const TopAbs_ShapeEnum TA,
                                            TopTools_IndexedDataMapOfShapeListOfShape &M) {
  TopExp::MapShapesAndUniqueAncestors(S, TS, TA, M);
}

inline std::unique_ptr<gp_Dir> TColgp_Array1OfDir_Value(const TColgp_Array1OfDir &array, Standard_Integer index) {
  return std::unique_ptr<gp_Dir>(new gp_Dir(array.Value(index)));
}

inline std::unique_ptr<gp_Pnt2d> TColgp_Array1OfPnt2d_Value(const TColgp_Array1OfPnt2d &array, Standard_Integer index) {
  return std::unique_ptr<gp_Pnt2d>(new gp_Pnt2d(array.Value(index)));
}

inline std::unique_ptr<gp_Pnt> TColgp_HArray1OfPnt_Value(const TColgp_HArray1OfPnt &array, Standard_Integer index) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(array.Value(index)));
}

inline void connect_edges_to_wires(HandleTopTools_HSequenceOfShape &edges, const Standard_Real toler,
                                   const Standard_Boolean shared, HandleTopTools_HSequenceOfShape &wires) {
  ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edges, toler, shared, wires);
}

inline std::unique_ptr<HandleTopTools_HSequenceOfShape> new_HandleTopTools_HSequenceOfShape() {
  auto sequence = new TopTools_HSequenceOfShape();
  auto handle = new opencascade::handle<TopTools_HSequenceOfShape>(sequence);

  return std::unique_ptr<HandleTopTools_HSequenceOfShape>(handle);
}

inline void TopTools_HSequenceOfShape_append(HandleTopTools_HSequenceOfShape &handle, const TopoDS_Shape &shape) {
  handle->Append(shape);
}

inline Standard_Integer TopTools_HSequenceOfShape_length(const HandleTopTools_HSequenceOfShape &handle) {
  return handle->Length();
}

inline const TopoDS_Shape &TopTools_HSequenceOfShape_value(const HandleTopTools_HSequenceOfShape &handle,
                                                           Standard_Integer index) {
  return handle->Value(index);
}

// BRep Algo API
inline std::unique_ptr<BRepAlgoAPI_BuilderAlgo>
cast_section_to_builderalgo(std::unique_ptr<BRepAlgoAPI_Section> section) {
  return section;
}
// namespace BRepAlgoAPI

// Bnd_Box
inline std::unique_ptr<Bnd_Box> Bnd_Box_ctor() { return std::unique_ptr<Bnd_Box>(new Bnd_Box()); }
inline std::unique_ptr<gp_Pnt> Bnd_Box_CornerMin(const Bnd_Box &box) {
  auto p = box.CornerMin();
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(p));
}
inline std::unique_ptr<gp_Pnt> Bnd_Box_CornerMax(const Bnd_Box &box) {
  auto p = box.CornerMax();
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(p));
}

// Interpolate a smooth B-spline wire through the given points (x0,y0,z0, x1,y1,z1, ...).
// Returns nullptr on failure or if fewer than 2 points are given.
// This is needed to convert a polyline rail into a smooth G2 spine for MakePipeShell,
// so the Frenet frame is well-defined everywhere and the profile rotates correctly.
inline std::unique_ptr<TopoDS_Wire> interpolate_points_to_wire(rust::Slice<const double> xyz) {
  int n = static_cast<int>(xyz.size()) / 3;
  if (n < 2) return nullptr;
  try {
    Handle(TColgp_HArray1OfPnt) pts = new TColgp_HArray1OfPnt(1, n);
    for (int i = 0; i < n; i++) {
      pts->SetValue(i + 1, gp_Pnt(xyz[3*i], xyz[3*i+1], xyz[3*i+2]));
    }
    GeomAPI_Interpolate interp(pts, /*IsPeriodic=*/false, /*Tolerance=*/1e-6);
    interp.Perform();
    if (!interp.IsDone()) return nullptr;
    Handle(Geom_BSplineCurve) curve = interp.Curve();
    TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(curve).Edge();
    TopoDS_Wire wire = BRepBuilderAPI_MakeWire(edge).Wire();
    return std::unique_ptr<TopoDS_Wire>(new TopoDS_Wire(wire));
  } catch (...) {
    return nullptr;
  }
}

// Safe MakePipe wrapper: returns null unique_ptr on OCCT exception instead of aborting.
inline std::unique_ptr<BRepOffsetAPI_MakePipe> try_BRepOffsetAPI_MakePipe_ctor(
    const TopoDS_Wire &spine,
    const TopoDS_Shape &profile) {
  try {
    return std::unique_ptr<BRepOffsetAPI_MakePipe>(
        new BRepOffsetAPI_MakePipe(spine, profile));
  } catch (...) {
    return std::unique_ptr<BRepOffsetAPI_MakePipe>(nullptr);
  }
}

// Safe MakePipe::Shape wrapper: returns null on exception (Shape() throws if !IsDone).
inline std::unique_ptr<TopoDS_Shape> try_BRepOffsetAPI_MakePipe_Shape(
    BRepOffsetAPI_MakePipe &pipe) {
  try {
    if (!pipe.IsDone()) return nullptr;
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(pipe.Shape()));
  } catch (...) {
    return nullptr;
  }
}

// Safe wire start-tangent extractor: fills (px,py,pz, tx,ty,tz) and returns true on success.
// Used to align the profile circle with the actual B-spline tangent (not the polyline approx).
inline bool wire_start_point_and_tangent(
    const TopoDS_Wire& wire,
    double& px, double& py, double& pz,
    double& tx, double& ty, double& tz) {
  try {
    TopExp_Explorer exp(wire, TopAbs_EDGE);
    if (!exp.More()) return false;
    TopoDS_Edge edge = TopoDS::Edge(exp.Current());
    Standard_Real first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
    if (curve.IsNull()) return false;
    gp_Pnt pnt;
    gp_Vec tan;
    curve->D1(first, pnt, tan);
    px = pnt.X(); py = pnt.Y(); pz = pnt.Z();
    Standard_Real len = tan.Magnitude();
    if (len < 1e-12) return false;
    tx = tan.X()/len; ty = tan.Y()/len; tz = tan.Z()/len;
    return true;
  } catch (...) { return false; }
}

// Safe MakePipeShell::Build wrapper: returns false on OCCT exception instead of aborting.
inline bool try_BRepOffsetAPI_MakePipeShell_Build(
    BRepOffsetAPI_MakePipeShell &pipe_shell,
    const Message_ProgressRange &progress) {
  try {
    pipe_shell.Build(progress);
    return pipe_shell.IsDone();
  } catch (...) {
    return false;
  }
}

// Safe Add wrapper: returns false on exception.
// WithContact=true  → moves profile to contact the spine start
// WithCorrection=true → rotates profile to align with the spine's Frenet frame
inline bool try_BRepOffsetAPI_MakePipeShell_Add(
    BRepOffsetAPI_MakePipeShell &pipe_shell,
    const TopoDS_Shape &profile) {
  try {
    pipe_shell.Add(profile, /*WithContact=*/true, /*WithCorrection=*/true);
    return true;
  } catch (...) {
    return false;
  }
}

// Safe Add wrapper WITHOUT contact/correction — profile is already pre-positioned at spine start.
// Use this when the profile has been manually placed at the rail start, perpendicular to tangent.
inline bool try_BRepOffsetAPI_MakePipeShell_Add_raw(
    BRepOffsetAPI_MakePipeShell &pipe_shell,
    const TopoDS_Shape &profile) {
  try {
    pipe_shell.Add(profile, /*WithContact=*/false, /*WithCorrection=*/false);
    return true;
  } catch (...) {
    return false;
  }
}

// WithContact=true, WithCorrection=false: moves profile to spine start but preserves its orientation.
// Use this when the user wants to control the profile's orientation but have OCCT position it on the spine.
inline bool try_BRepOffsetAPI_MakePipeShell_Add_contact(
    BRepOffsetAPI_MakePipeShell &pipe_shell,
    const TopoDS_Shape &profile) {
  try {
    pipe_shell.Add(profile, /*WithContact=*/true, /*WithCorrection=*/false);
    return true;
  } catch (...) {
    return false;
  }
}

// Safe Shape wrapper: returns null on exception (e.g. shape not built).
inline std::unique_ptr<TopoDS_Shape> try_BRepOffsetAPI_MakePipeShell_Shape(
    BRepOffsetAPI_MakePipeShell &pipe_shell) {
  try {
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(pipe_shell.Shape()));
  } catch (...) {
    return nullptr;
  }
}

// Safe MakeSolid wrapper: returns false on exception.
inline bool try_BRepOffsetAPI_MakePipeShell_MakeSolid(
    BRepOffsetAPI_MakePipeShell &pipe_shell) {
  try {
    return pipe_shell.MakeSolid();
  } catch (...) {
    return false;
  }
}

// Safe circle wire builder: creates a closed circle wire centered at (cx,cy,cz) with
// normal (nx,ny,nz) and the given radius.  Returns null on any exception.
inline std::unique_ptr<TopoDS_Wire> try_make_circle_wire(
    double cx, double cy, double cz,
    double nx, double ny, double nz,
    double radius) {
  try {
    gp_Ax2 ax(gp_Pnt(cx, cy, cz), gp_Dir(nx, ny, nz));
    gp_Circ circ(ax, radius);
    TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(circ).Edge();
    BRepBuilderAPI_MakeWire mw;
    mw.Add(edge);
    if (!mw.IsDone()) return nullptr;
    return std::unique_ptr<TopoDS_Wire>(new TopoDS_Wire(mw.Wire()));
  } catch (...) {
    return nullptr;
  }
}

// Safe MakePipeShell constructor: returns null on any exception.
inline std::unique_ptr<BRepOffsetAPI_MakePipeShell> try_BRepOffsetAPI_MakePipeShell_ctor(
    const TopoDS_Wire &spine) {
  try {
    return std::unique_ptr<BRepOffsetAPI_MakePipeShell>(new BRepOffsetAPI_MakePipeShell(spine));
  } catch (...) {
    return nullptr;
  }
}

// Safe SetDiscreteMode wrapper: uses per-vertex discrete trihedra instead of a continuous
// Frenet frame. Required for piecewise-linear (polyline/composite-of-lines) spines so that
// the profile rotates sharply at each joint rather than not rotating at all.
inline bool try_BRepOffsetAPI_MakePipeShell_SetDiscreteMode(
    BRepOffsetAPI_MakePipeShell &pipe_shell) {
  try {
    pipe_shell.SetDiscreteMode();
    return true;
  } catch (...) {
    return false;
  }
}

// Safe SetMode(Wire, bool) wrapper: sets an auxiliary spine wire to control
// profile orientation/scaling evolution along the main spine (two-rail sweep).
// curvilinear_equivalence=true uses curvilinear abscissa matching between spines.
inline bool try_BRepOffsetAPI_MakePipeShell_SetMode_Wire(
    BRepOffsetAPI_MakePipeShell &pipe_shell,
    const TopoDS_Wire &auxiliary_spine,
    bool curvilinear_equivalence) {
  try {
    pipe_shell.SetMode(auxiliary_spine, curvilinear_equivalence);
    return true;
  } catch (...) {
    return false;
  }
}

// ForceApproxC1: ask OCCT to attempt a C1 surface where it would produce C0.
inline void try_BRepOffsetAPI_MakePipeShell_SetForceApproxC1(
    BRepOffsetAPI_MakePipeShell &pipe_shell) {
  try { pipe_shell.SetForceApproxC1(true); } catch (...) {}
}

// RightCorner transition: produces clean right-angle joints at non-G1 spine corners
// (polyline/segmented rails). Avoids the faceted "cut-off" look at each joint.
inline void try_BRepOffsetAPI_MakePipeShell_SetTransitionMode_Right(
    BRepOffsetAPI_MakePipeShell &pipe_shell) {
  try {
    pipe_shell.SetTransitionMode(BRepBuilderAPI_RightCorner);
  } catch (...) {}
}

// RoundCorner transition: inserts a small arc-blend at each non-G1 spine corner
// instead of a sharp mitre. Use this when the rail has obtuse kinks (>~90°),
// where RightCorner's mitre line projects far outside the rail bounds and
// produces "extra ribbon" geometry shooting into space.
inline void try_BRepOffsetAPI_MakePipeShell_SetTransitionMode_RoundCorner(
    BRepOffsetAPI_MakePipeShell &pipe_shell) {
  try {
    pipe_shell.SetTransitionMode(BRepBuilderAPI_RoundCorner);
  } catch (...) {}
}

// Fixed-binormal mode: constrains profile orientation by locking a binormal direction
// in world space for the entire sweep. Profile keeps its global "up" axis (e.g. world Z)
// no matter how the rail bends. Eliminates the per-segment trihedron flip that
// SetDiscreteMode produces on polyline rails with sharp/near-180 corners (which
// manifests as the "second half of the sweep mirrored" symptom).
//
// Maps to OCCT's BRepOffsetAPI_MakePipeShell::SetMode(const gp_Dir&). Returns false
// on OCCT exception or invalid direction (zero-length vector).
inline bool try_BRepOffsetAPI_MakePipeShell_SetMode_FixedBinormal(
    BRepOffsetAPI_MakePipeShell &pipe_shell,
    double bx, double by, double bz) {
  try {
    gp_Dir binormal(bx, by, bz);
    pipe_shell.SetMode(binormal);
    return true;
  } catch (...) {
    return false;
  }
}

// Safe BRepBuilderAPI_MakeWire::Wire() wrapper.
// BRepBuilderAPI_MakeWire::Wire() throws StdFail_NotDone when IsDone() is false.
// Returns null instead of throwing.
inline std::unique_ptr<TopoDS_Wire> try_BRepBuilderAPI_MakeWire_Wire(
    BRepBuilderAPI_MakeWire &mw) {
  try {
    if (!mw.IsDone()) return nullptr;
    return std::unique_ptr<TopoDS_Wire>(new TopoDS_Wire(mw.Wire()));
  } catch (...) {
    return nullptr;
  }
}

// Safe BRepOffsetAPI_MakeOffsetShape wrapper — uniformly offsets a solid or shell.
// Positive offset expands, negative shrinks. Returns null on OCCT exception or failure.
inline std::unique_ptr<TopoDS_Shape> try_MakeOffsetShape(
    const TopoDS_Shape &shape,
    double offset,
    double tolerance) {
  try {
    BRepOffsetAPI_MakeOffsetShape offset_maker;
    offset_maker.PerformByJoin(shape, offset, tolerance);
    if (!offset_maker.IsDone()) return nullptr;
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(offset_maker.Shape()));
  } catch (...) {
    return nullptr;
  }
}

// Safe MakeThickSolidByJoin wrapper: returns null on OCCT exception instead of aborting.
inline std::unique_ptr<TopoDS_Shape> try_MakeThickSolidByJoin(
    const TopoDS_Shape &shape,
    const TopTools_ListOfShape &closing_faces,
    double offset,
    double tolerance) {
  try {
    BRepOffsetAPI_MakeThickSolid solid_maker;
    solid_maker.MakeThickSolidByJoin(shape, closing_faces, offset, tolerance);
    if (!solid_maker.IsDone()) return nullptr;
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(solid_maker.Shape()));
  } catch (...) {
    return nullptr;
  }
}

// Safe MakeThickSolidBySimple wrapper: thickens an open shell into a solid.
// Returns null on OCCT exception instead of aborting.
inline std::unique_ptr<TopoDS_Shape> try_MakeThickSolidBySimple(
    const TopoDS_Shape &shape,
    double offset) {
  try {
    BRepOffsetAPI_MakeThickSolid solid_maker;
    solid_maker.MakeThickSolidBySimple(shape, offset);
    if (!solid_maker.IsDone()) return nullptr;
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(solid_maker.Shape()));
  } catch (...) {
    return nullptr;
  }
}

// Safe BRepFilletAPI_MakeFillet wrapper: builds, checks IsDone, returns null on any exception.
inline std::unique_ptr<TopoDS_Shape> try_BRepFilletAPI_MakeFillet_Shape(
    BRepFilletAPI_MakeFillet &fillet) {
  try {
    fillet.Build(Message_ProgressRange());
    if (!fillet.IsDone()) return nullptr;
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(fillet.Shape()));
  } catch (...) {
    return nullptr;
  }
}

// Safe BRepFilletAPI_MakeChamfer wrapper: builds, checks IsDone, returns null on any exception.
inline std::unique_ptr<TopoDS_Shape> try_BRepFilletAPI_MakeChamfer_Shape(
    BRepFilletAPI_MakeChamfer &chamfer) {
  try {
    chamfer.Build(Message_ProgressRange());
    if (!chamfer.IsDone()) return nullptr;
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(chamfer.Shape()));
  } catch (...) {
    return nullptr;
  }
}

// Safe fillet add_edge: returns false on exception (e.g. degenerate edge).
inline bool try_BRepFilletAPI_MakeFillet_AddEdge(
    BRepFilletAPI_MakeFillet &fillet,
    double radius,
    const TopoDS_Edge &edge) {
  try {
    fillet.Add(radius, edge);
    return true;
  } catch (...) {
    return false;
  }
}

// Safe chamfer add_edge: returns false on exception.
inline bool try_BRepFilletAPI_MakeChamfer_AddEdge(
    BRepFilletAPI_MakeChamfer &chamfer,
    double distance,
    const TopoDS_Edge &edge) {
  try {
    chamfer.Add(distance, edge);
    return true;
  } catch (...) {
    return false;
  }
}

// Safe BRepBuilderAPI_MakeFace::Add wrapper — adds an inner wire (hole) to an existing face.
// Reconstructs MakeFace from the face, calls Add(wire), returns new face or null on failure.
// Inner wires must be REVERSED relative to the outer wire for OCCT to treat them as holes.
// We try the wire as-given first; if that doesn't reduce the surface area, we reverse it.
inline std::unique_ptr<TopoDS_Face> try_AddWireToFace(
    const TopoDS_Face &face,
    const TopoDS_Wire &wire) {
  try {
    // Compute original face area for comparison
    GProp_GProps original_props;
    BRepGProp::SurfaceProperties(face, original_props);
    double original_area = original_props.Mass();

    // Try adding wire as-is
    BRepBuilderAPI_MakeFace mf(face);
    mf.Add(wire);
    if (mf.IsDone()) {
      TopoDS_Face result = mf.Face();
      GProp_GProps result_props;
      BRepGProp::SurfaceProperties(result, result_props);
      double result_area = result_props.Mass();
      // If area decreased, the inner wire was correctly oriented as a hole
      if (result_area < original_area - 1e-6) {
        return std::unique_ptr<TopoDS_Face>(new TopoDS_Face(result));
      }
    }

    // Try with reversed wire
    TopoDS_Wire reversed = TopoDS::Wire(wire.Reversed());
    BRepBuilderAPI_MakeFace mf2(face);
    mf2.Add(reversed);
    if (mf2.IsDone()) {
      TopoDS_Face result2 = mf2.Face();
      GProp_GProps result2_props;
      BRepGProp::SurfaceProperties(result2, result2_props);
      double result2_area = result2_props.Mass();
      if (result2_area < original_area - 1e-6) {
        return std::unique_ptr<TopoDS_Face>(new TopoDS_Face(result2));
      }
    }

    // Fallback: return whatever the first attempt produced
    if (mf.IsDone()) {
      return std::unique_ptr<TopoDS_Face>(new TopoDS_Face(mf.Face()));
    }
    return nullptr;
  } catch (...) {
    return nullptr;
  }
}

// ShapeFix_Face::FixOrientation — OCCT's Shape Healing auto-orients wires on a face.
// Outer wire → CCW, inner wires (holes) → CW.  Reference:
//   • OCCT docs: ShapeFix_Face Class — "If the face has several wires, they are
//     oriented to lay one outside another (if possible)."
//   • Forum: https://dev.opencascade.org/content/orientation-faceswires
//   • Signed-area winding test: https://dev.opencascade.org/content/getting-wire-direction
//     "Create a probing face with a wire on the given surface and determine its area
//      using BRepGProp::SurfaceProperties. If area is positive the wire is CCW."
//   • Generalized Winding Number (research):
//     https://arxiv.org/html/2403.17371v1  (Robust Containment Queries over
//     Collections of Rational Parametric Curves via Generalized Winding Numbers)
//
// Returns a new face with corrected wire orientations, or null on failure.
inline std::unique_ptr<TopoDS_Face> ShapeFix_Face_FixOrientation(
    const TopoDS_Face &face) {
  try {
    Handle(ShapeFix_Face) sff = new ShapeFix_Face(face);
    sff->FixOrientation();
    if (sff->Face().IsNull()) return nullptr;
    return std::unique_ptr<TopoDS_Face>(new TopoDS_Face(sff->Face()));
  } catch (...) {
    return nullptr;
  }
}

// Bulk edge classification by face adjacency.
// Returns flat-packed: [n_naked, n_interior, n_non_manifold, naked_idx_0, ..., interior_idx_0, ..., nm_idx_0, ...]
// Edge indices are in TopExp_Explorer(TopAbs_EDGE) order (same as shape.edges()).
// Naked=1 adjacent face, Interior=2, NonManifold=3+.
inline rust::Vec<int32_t> brep_classify_edges(const TopoDS_Shape &shape) {
  rust::Vec<int32_t> result;
  try {
    TopTools_IndexedDataMapOfShapeListOfShape edge_face_map;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_face_map);

    std::vector<int32_t> naked, interior, non_manifold;
    int32_t idx = 0;
    for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), idx++) {
      int map_idx = edge_face_map.FindIndex(exp.Current());
      int face_count = (map_idx > 0) ? edge_face_map(map_idx).Extent() : 0;
      if (face_count <= 1) naked.push_back(idx);
      else if (face_count == 2) interior.push_back(idx);
      else non_manifold.push_back(idx);
    }

    result.push_back(static_cast<int32_t>(naked.size()));
    result.push_back(static_cast<int32_t>(interior.size()));
    result.push_back(static_cast<int32_t>(non_manifold.size()));
    for (auto i : naked) result.push_back(i);
    for (auto i : interior) result.push_back(i);
    for (auto i : non_manifold) result.push_back(i);
  } catch (...) {
    // On failure, return empty classification
    result.push_back(0);
    result.push_back(0);
    result.push_back(0);
  }
  return result;
}

// BRepBndLib
inline void BRepBndLib_Add(const TopoDS_Shape &shape, Bnd_Box &box, const Standard_Boolean useTriangulation) {
  BRepBndLib::Add(shape, box, useTriangulation);
}

// Tight box computed by sampling the real curves/surfaces. Unlike Add(), the
// result does not depend on whether the shape happens to carry a triangulation:
// Add() falls back to the B-spline control hull when there is none, which on a
// swept branch is 22-28x the true extent.
inline void BRepBndLib_AddOptimal(const TopoDS_Shape &shape, Bnd_Box &box,
                                  const Standard_Boolean useTriangulation,
                                  const Standard_Boolean useShapeTolerance) {
  BRepBndLib::AddOptimal(shape, box, useTriangulation, useShapeTolerance);
}

// ---------------------------------------------------------------------------
// NURBS readback — turning a face back into the numbers that define it
// ---------------------------------------------------------------------------
//
// Everything above this line was built to *author* geometry (construct a Bezier
// surface, build a face from it) or to *mesh* it. Nothing was built to read a
// surface's own definition back out, which is what an evaluator outside OCCT needs.
//
// Two rules shape this whole section:
//
// 1. **Convert the face, not the surface.** The obvious move is
//    BRep_Tool::Surface(face) followed by GeomConvert::SurfaceToBSplineSurface. It is
//    wrong, and quietly: the conversion may reparametrize, while BRepTools::UVBounds
//    and the p-curves still speak the *original* surface's parameters. The surface,
//    its domain and its trim loops would then be three answers to three different
//    questions. BRepBuilderAPI_NurbsConvert rebuilds the face — surface and p-curves
//    together — so reading all three off the converted face is self-consistent by
//    construction.
//
// 2. **Bulk arrays cross once.** Poles are the only O(n) quantity here, and a
//    per-pole call would mean one heap allocation and one FFI hop per control point
//    per face. They come back flat-packed, the same idiom brep_classify_edges uses.
//    The scalars stay as named accessors: nine consecutive numbers as positional
//    arguments transpose silently and fail as a wrong *surface*, not a compile error.
//
// Every entry point here catches. An OCCT exception crossing this boundary is a
// SIGABRT, not an Err.

typedef opencascade::handle<Geom_BSplineSurface> HandleGeomBSplineSurface;

// The face rebuilt on B-spline geometry, or null. Null is a real answer — a face
// OCCT declines to convert is one this pipeline cannot render, and the caller falls
// back to the mesher rather than guessing.
inline std::unique_ptr<TopoDS_Face> nurbs_convert_face(const TopoDS_Face &face) {
  try {
    BRepBuilderAPI_NurbsConvert converter(face, Standard_True);
    const TopoDS_Shape &out = converter.Shape();
    if (out.IsNull() || out.ShapeType() != TopAbs_FACE) {
      return std::unique_ptr<TopoDS_Face>();
    }
    return std::unique_ptr<TopoDS_Face>(new TopoDS_Face(TopoDS::Face(out)));
  } catch (...) {
    return std::unique_ptr<TopoDS_Face>();
  }
}

// The B-spline surface under a face, or null.
//
// Deliberately does not convert as a fallback, because a conversion here would produce
// a surface whose parameters the caller's UV bounds and p-curves do not share -- see
// rule 1 above. That rule is about where the three answers come from, not about which
// face is passed: reading surface, UV bounds and p-curves all off one face satisfies it
// whether or not that face has been through nurbs_convert_face.
//
// So this is called on both, and the DownCast is what distinguishes them. On the
// original face it succeeds exactly when the face already carries a
// Geom_BSplineSurface -- which is the same condition
// BRepTools_NurbsConvertModification::NewSurface tests before declining to build a new
// one, so a success here means the conversion would have handed this very handle back.
// Face::to_nurbs uses that to skip it.
//
// Safe to call on anything: a null surface, a DownCast that fails and an OCCT throw all
// return null, so probing a plane or a cone costs a null check rather than an abort.
inline std::unique_ptr<HandleGeomBSplineSurface> bspline_surface_of_face(const TopoDS_Face &face) {
  try {
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) {
      return std::unique_ptr<HandleGeomBSplineSurface>();
    }
    Handle(Geom_BSplineSurface) bspline = Handle(Geom_BSplineSurface)::DownCast(surface);
    if (bspline.IsNull()) {
      return std::unique_ptr<HandleGeomBSplineSurface>();
    }
    return std::unique_ptr<HandleGeomBSplineSurface>(
        new opencascade::handle<Geom_BSplineSurface>(bspline));
  } catch (...) {
    return std::unique_ptr<HandleGeomBSplineSurface>();
  }
}

// The single exception to rule 1, and it is narrow on purpose.
//
// BRepTools_NurbsConvertModification::NewSurface returns Standard_False for a
// Geom_BezierSurface (BRepTools_NurbsConvertModification.cxx:247) -- OCCT considers Bezier
// to already *be* NURBS and declines to build a new surface. So nurbs_convert_face hands
// back an unmodified face, bspline_surface_of_face's DownCast fails on it too, and a Bezier
// face has no B-spline route at all. It renders as nothing.
//
// Converting the surface directly is what rule 1 forbids, and the reason it forbids it is
// reparametrization: a conversion that moves the parameters leaves UVBounds and the p-curves
// describing a domain the new surface does not have. That reason does not apply here, and
// not by luck --
//
//   * GeomConvert::SurfaceToBSplineSurface's Bezier branch (GeomConvert_1.cxx:822) sets knots
//     {0, 1} with multiplicities degree+1 at both ends and copies poles and weights verbatim.
//     A Bezier is already parametrized on [0,1]^2. The result is the same surface written a
//     different way, not a fit.
//   * And the conversion *declining* is exactly what guarantees the rest is untouched: since
//     NurbsConvert never rebuilt the face, its UV bounds and p-curves still speak the Bezier's
//     own parameters -- which are the parameters this returns.
//
// Hence the DynamicType gate rather than an IsKind or a DownCast. Anything that is not
// literally a Geom_BezierSurface gets null and falls back to the general path, where rule 1
// still holds with no exceptions.
inline std::unique_ptr<HandleGeomBSplineSurface> bspline_from_bezier_face(const TopoDS_Face &face) {
  try {
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull() || surface->DynamicType() != STANDARD_TYPE(Geom_BezierSurface)) {
      return std::unique_ptr<HandleGeomBSplineSurface>();
    }
    Handle(Geom_BSplineSurface) bspline = GeomConvert::SurfaceToBSplineSurface(surface);
    if (bspline.IsNull()) {
      return std::unique_ptr<HandleGeomBSplineSurface>();
    }
    return std::unique_ptr<HandleGeomBSplineSurface>(
        new opencascade::handle<Geom_BSplineSurface>(bspline));
  } catch (...) {
    return std::unique_ptr<HandleGeomBSplineSurface>();
  }
}

inline bool HandleGeomBSplineSurface_IsNull(const HandleGeomBSplineSurface &surface) {
  return surface.IsNull();
}

// Scalars, named rather than packed. See rule 2.
inline int32_t bspline_u_degree(const HandleGeomBSplineSurface &s) { return s->UDegree(); }
inline int32_t bspline_v_degree(const HandleGeomBSplineSurface &s) { return s->VDegree(); }
inline int32_t bspline_nb_u_poles(const HandleGeomBSplineSurface &s) { return s->NbUPoles(); }
inline int32_t bspline_nb_v_poles(const HandleGeomBSplineSurface &s) { return s->NbVPoles(); }
inline int32_t bspline_nb_u_knots(const HandleGeomBSplineSurface &s) { return s->NbUKnots(); }
inline int32_t bspline_nb_v_knots(const HandleGeomBSplineSurface &s) { return s->NbVKnots(); }
inline bool bspline_is_u_periodic(const HandleGeomBSplineSurface &s) { return s->IsUPeriodic(); }
inline bool bspline_is_v_periodic(const HandleGeomBSplineSurface &s) { return s->IsVPeriodic(); }

// Whether the weights mean anything. A polynomial surface still answers Weight(i,j)
// -- with 1.0 -- so this is not an optimisation: it is the difference between reading
// a value and reading a placeholder, and the consumer stores None rather than a vector
// of ones precisely so the two cannot be confused later.
inline bool bspline_is_u_rational(const HandleGeomBSplineSurface &s) { return s->IsURational(); }
inline bool bspline_is_v_rational(const HandleGeomBSplineSurface &s) { return s->IsVRational(); }

// NbUPoles * NbVPoles * 4 doubles: x, y, z, w per pole, **u-major** -- pole(i,j) is at
// (i * NbVPoles + j) * 4. OCCT's own arrays are 1-based and column-major-ish by
// convention; the +1s and the ordering are settled here, once, rather than at every
// call site. Empty on failure.
//
// The weight is always written. On a polynomial surface it is OCCT's 1.0, and the flag
// above is what says whether to believe it.
inline rust::Vec<double> bspline_poles(const HandleGeomBSplineSurface &s) {
  rust::Vec<double> out;
  try {
    const Standard_Integer nu = s->NbUPoles();
    const Standard_Integer nv = s->NbVPoles();
    out.reserve(static_cast<size_t>(nu) * static_cast<size_t>(nv) * 4);
    for (Standard_Integer i = 1; i <= nu; ++i) {
      for (Standard_Integer j = 1; j <= nv; ++j) {
        const gp_Pnt p = s->Pole(i, j);
        out.push_back(p.X());
        out.push_back(p.Y());
        out.push_back(p.Z());
        out.push_back(s->Weight(i, j));
      }
    }
  } catch (...) {
    rust::Vec<double> empty;
    return empty;
  }
  return out;
}

// The **distinct** knots, not the flat sequence. OCCT stores knots compressed, and the
// multiplicities below are the other half; expanding them is the consumer's job and is
// the single most dangerous step in this whole readback -- an off-by-one multiplicity
// produces a surface that is subtly wrong near every interior knot and looks right.
inline rust::Vec<double> bspline_u_knots(const HandleGeomBSplineSurface &s) {
  rust::Vec<double> out;
  try {
    const Standard_Integer n = s->NbUKnots();
    out.reserve(static_cast<size_t>(n));
    for (Standard_Integer i = 1; i <= n; ++i) out.push_back(s->UKnot(i));
  } catch (...) {
    rust::Vec<double> empty;
    return empty;
  }
  return out;
}

inline rust::Vec<double> bspline_v_knots(const HandleGeomBSplineSurface &s) {
  rust::Vec<double> out;
  try {
    const Standard_Integer n = s->NbVKnots();
    out.reserve(static_cast<size_t>(n));
    for (Standard_Integer i = 1; i <= n; ++i) out.push_back(s->VKnot(i));
  } catch (...) {
    rust::Vec<double> empty;
    return empty;
  }
  return out;
}

inline rust::Vec<int32_t> bspline_u_mults(const HandleGeomBSplineSurface &s) {
  rust::Vec<int32_t> out;
  try {
    const Standard_Integer n = s->NbUKnots();
    out.reserve(static_cast<size_t>(n));
    for (Standard_Integer i = 1; i <= n; ++i) out.push_back(s->UMultiplicity(i));
  } catch (...) {
    rust::Vec<int32_t> empty;
    return empty;
  }
  return out;
}

inline rust::Vec<int32_t> bspline_v_mults(const HandleGeomBSplineSurface &s) {
  rust::Vec<int32_t> out;
  try {
    const Standard_Integer n = s->NbVKnots();
    out.reserve(static_cast<size_t>(n));
    for (Standard_Integer i = 1; i <= n; ++i) out.push_back(s->VMultiplicity(i));
  } catch (...) {
    rust::Vec<int32_t> empty;
    return empty;
  }
  return out;
}

// [u_min, u_max, v_min, v_max] for the face's own domain, or empty.
//
// Not the surface's natural bounds: a face is a *bounded* region of a surface, and on
// a plane the surface's own bounds are infinite. This is the sampling rectangle.
inline rust::Vec<double> face_uv_bounds(const TopoDS_Face &face) {
  rust::Vec<double> out;
  try {
    Standard_Real u_min = 0.0, u_max = 0.0, v_min = 0.0, v_max = 0.0;
    BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);
    out.push_back(u_min);
    out.push_back(u_max);
    out.push_back(v_min);
    out.push_back(v_max);
  } catch (...) {
    rust::Vec<double> empty;
    return empty;
  }
  return out;
}

// ---------------------------------------------------------------------------
// The oracle: OCCT evaluating the same surface
// ---------------------------------------------------------------------------
//
// Nothing in the readback above is self-checking. A transposed pole grid, a knot array
// off by one multiplicity, a Weight(i,j) read (j,i) -- each produces a surface that is
// smooth, plausible and wrong, and the first thing that would notice is a picture.
//
// These two let a test ask OCCT what the answer is, at f64, so a disagreement means the
// readback is wrong because it cannot mean anything else. Deliberately the *surface*
// adaptor and not BRep_Tool::Triangulation: a triangulation is deflection-limited, so
// disagreements against it are dominated by the mesher's tolerance rather than by
// anyone's arithmetic, and the threshold ends up tuned until it passes.

// Null on failure, which an out-of-domain parameter is a real way to reach. The caller
// gets an Option rather than a process abort.
inline std::unique_ptr<gp_Pnt> BRepAdaptor_Surface_value(const BRepAdaptor_Surface &surface, double u,
                                                         double v) {
  try {
    return std::unique_ptr<gp_Pnt>(new gp_Pnt(surface.Value(u, v)));
  } catch (...) {
    return std::unique_ptr<gp_Pnt>();
  }
}

// 9 doubles: position, dS/du, dS/dv. Empty on failure.
//
// Packed rather than three calls because D1 computes all three together and splitting
// it would evaluate the surface three times to return one answer.
inline rust::Vec<double> BRepAdaptor_Surface_d1(const BRepAdaptor_Surface &surface, double u, double v) {
  rust::Vec<double> out;
  try {
    gp_Pnt p;
    gp_Vec d1u, d1v;
    surface.D1(u, v, p, d1u, d1v);
    out.push_back(p.X());
    out.push_back(p.Y());
    out.push_back(p.Z());
    out.push_back(d1u.X());
    out.push_back(d1u.Y());
    out.push_back(d1u.Z());
    out.push_back(d1v.X());
    out.push_back(d1v.Y());
    out.push_back(d1v.Z());
  } catch (...) {
    rust::Vec<double> empty;
    return empty;
  }
  return out;
}

// [u, v, distance] of the closest point on the surface, or empty.
//
// This is the *conversion* oracle, and it is a different shape from the two above on
// purpose. BRepBuilderAPI_NurbsConvert may reparametrize, so S_orig(u,v) and
// S_conv(u,v) can both be correct and disagree -- an equality at a shared parameter
// would fail on a conversion that did its job. Asking how far a point *is* from the
// surface never assumes the two agree about (u,v).
//
// GeomAPI_ProjectPointOnSurf's own accessors throw StdFail_NotDone when nothing was
// found, and the existing binding of LowerDistanceParameters is a direct method call
// with no catch -- so it is bound again here rather than reused, because a projection
// that finds nothing is a normal outcome for this caller and must not abort the
// process.
inline rust::Vec<double> project_point_on_surface(const HandleGeomSurface &surface,
                                                  const gp_Pnt &point) {
  rust::Vec<double> out;
  try {
    GeomAPI_ProjectPointOnSurf projector(point, surface);
    if (!projector.IsDone() || projector.NbPoints() < 1) {
      rust::Vec<double> empty;
      return empty;
    }
    Standard_Real u = 0.0, v = 0.0;
    projector.LowerDistanceParameters(u, v);
    out.push_back(u);
    out.push_back(v);
    out.push_back(projector.LowerDistance());
  } catch (...) {
    rust::Vec<double> empty;
    return empty;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Trimming: the p-curves that cut a face out of its surface
// ---------------------------------------------------------------------------

// True for an edge that exists only in parameter space -- the collapsed side of a
// sphere's pole row, say. It has a p-curve but no 3D curve, and sampling it as though
// it bounded something produces a spurious loop through the degenerate point.
inline bool BRep_Tool_Degenerated(const TopoDS_Edge &edge) {
  try {
    return BRep_Tool::Degenerated(edge) == Standard_True;
  } catch (...) {
    return false;
  }
}

// The 2D curve of an edge **as seen by one face**. The pair matters: a seam edge
// belongs to the same face twice and has a different p-curve each time, which is
// exactly what closes the loop around a periodic surface.
//
// Null when the edge carries no p-curve on this face.
inline std::unique_ptr<HandleGeom2d_Curve> BRep_Tool_CurveOnSurface(const TopoDS_Edge &edge,
                                                                    const TopoDS_Face &face) {
  try {
    Standard_Real first = 0.0, last = 0.0;
    Handle(Geom2d_Curve) curve = BRep_Tool::CurveOnSurface(edge, face, first, last);
    if (curve.IsNull()) {
      return std::unique_ptr<HandleGeom2d_Curve>();
    }
    return std::unique_ptr<HandleGeom2d_Curve>(new opencascade::handle<Geom2d_Curve>(curve));
  } catch (...) {
    return std::unique_ptr<HandleGeom2d_Curve>();
  }
}

// [first, last] for the same edge/face pair, or empty.
//
// A second call rather than out-parameters: cxx would carry these as &mut f64, and the
// two-call cost is per *edge*, not per sample, against a p-curve OCCT has already
// cached on the edge.
inline rust::Vec<double> BRep_Tool_CurveOnSurface_range(const TopoDS_Edge &edge, const TopoDS_Face &face) {
  rust::Vec<double> out;
  try {
    Standard_Real first = 0.0, last = 0.0;
    Handle(Geom2d_Curve) curve = BRep_Tool::CurveOnSurface(edge, face, first, last);
    if (curve.IsNull()) {
      rust::Vec<double> empty;
      return empty;
    }
    out.push_back(first);
    out.push_back(last);
  } catch (...) {
    rust::Vec<double> empty;
    return empty;
  }
  return out;
}

// Sample a p-curve. The Geom2d_* types were bound for construction only -- they could
// be built and passed to a face builder, never read back. Null on failure.
inline std::unique_ptr<gp_Pnt2d> Geom2d_Curve_value(const HandleGeom2d_Curve &curve, double t) {
  try {
    return std::unique_ptr<gp_Pnt2d>(new gp_Pnt2d(curve->Value(t)));
  } catch (...) {
    return std::unique_ptr<gp_Pnt2d>();
  }
}

// Walk a wire's edges in CONNECTION order, which TopExp_Explorer does not promise.
//
// This is the whole reason for the type. TopExp_Explorer visits the edges a wire
// contains, in whatever order the TShape stores them; BRepTools_WireExplorer walks the
// chain, vertex to vertex, and hands back each edge oriented as the wire traverses it.
// For a trim loop those are not interchangeable: a loop assembled in explorer order is
// closed only by coincidence, and the winding-number test that consumes it reads a
// scrambled polygon as a plausible, wrong region.
//
// Face-aware on purpose. The two-argument constructor is what makes a SEAM edge behave:
// a seam belongs to the same face twice with a different p-curve each time, and only
// the (wire, face) form knows which traversal it is on.
//
// Null when OCCT threw -- a wire the explorer cannot start on is a real outcome, not a
// reason to abort the process.
inline std::unique_ptr<BRepTools_WireExplorer> BRepTools_WireExplorer_ctor(const TopoDS_Wire &wire,
                                                                          const TopoDS_Face &face) {
  try {
    return std::unique_ptr<BRepTools_WireExplorer>(new BRepTools_WireExplorer(wire, face));
  } catch (...) {
    return std::unique_ptr<BRepTools_WireExplorer>();
  }
}

// The edge the explorer is on, carrying its orientation within the wire.
inline std::unique_ptr<TopoDS_Edge> WireExplorerCurrentEdge(const BRepTools_WireExplorer &explorer) {
  return std::unique_ptr<TopoDS_Edge>(new TopoDS_Edge(explorer.Current()));
}

// BRepTools::OuterWire with a catch, and null rather than an empty wire when there is
// none.
//
// Deliberately a second entry point rather than a fix to `outer_wire` above: that one
// allocates unconditionally and four callers in this crate rely on it returning a wire,
// so making it nullable is a change to their contract and not to this one. OuterWire
// raises on a face carrying no wires, and an OCCT throw crossing the FFI uncaught is a
// SIGABRT rather than an Err -- which for a caller that is *asking* whether a face has
// an outer wire is the wrong answer to a reasonable question.
inline std::unique_ptr<TopoDS_Wire> face_outer_wire(const TopoDS_Face &face) {
  try {
    TopoDS_Wire wire = BRepTools::OuterWire(face);
    if (wire.IsNull()) {
      return std::unique_ptr<TopoDS_Wire>();
    }
    return std::unique_ptr<TopoDS_Wire>(new TopoDS_Wire(wire));
  } catch (...) {
    return std::unique_ptr<TopoDS_Wire>();
  }
}
