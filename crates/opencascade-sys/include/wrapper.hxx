#include "rust/cxx.h"
#include <BOPAlgo_GlueEnum.hxx>
#include <BRepAdaptor_Curve.hxx>
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
#include <Geom_BezierCurve.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <IGESControl_Reader.hxx>
#include <IGESControl_Writer.hxx>
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
