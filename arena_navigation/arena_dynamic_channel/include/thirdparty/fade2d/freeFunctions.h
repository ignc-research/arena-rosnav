// Copyright (C) Geom Software e.U, Bernhard Kornberger, Graz/Austria
//
// This file is part of the Fade2D library. The student license is free
// of charge and covers personal non-commercial research. Licensees
// holding a commercial license may use this file in accordance with
// the Commercial License Agreement.
//
// This software is provided AS IS with NO WARRANTY OF ANY KIND,
// INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE.
//
// Please contact the author if any conditions of this licensing are
// not clear to you.
//
// Author: Bernhard Kornberger, bkorn (at) geom.at
// http://www.geom.at



#pragma once
#include "Point2.h"
#include "Segment2.h"
#include "Edge2.h"
#include <vector>


#if GEOM_PSEUDO3D==GEOM_TRUE
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined
#endif

class Mesh3;

/** \defgroup tools Tools
 *  @{
 */

/** \brief Points-to-Polyline
 *
 * Turns a vector of points (p0,p1,p2,...pm,pn) into a vector of
 * segments ((p0,p1),(p1,p2),...,(pm,pn)). In case that bClose is
 * true an additional segment (pn,p0) is constructed. Degenerate
 * segments are ignored. Selfintersections of the polyline are
 * not checked.
 *
 * @param [in] vInPoints
 * @param [in] bClose specifies whether a closing segment shall be constructed
 * @param [out] vOutSegments is where the output segments are stored
 *
 */
CLASS_DECLSPEC
void pointsToPolyline(std::vector<Point2>& vInPoints,bool bClose,std::vector<Segment2>& vOutSegments);



/** \brief isSimplePolygon
 *
 * @param [in] vSegments specifies segments to be checked. Degenerate
 * segments (0-length) are ignored.
 * @return true when \p vSegments contains a closed polygon without
 * selfintersections. False otherwise.
 *
 */
CLASS_DECLSPEC
bool isSimplePolygon(std::vector<Segment2>& vSegments);

#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Get normal vector
 *
* Returns the normalized normal vector of the triangle defined by the
* three input points \p p0, \p p1, \p p2.
*
* @param [in] p0,p1,p2 When these points are counterclockwise (CCW) oriented
* then the resulting normal vector points towards the viewer.
*
* @param [out] bOK returns true for valid results. When the plane
* defined by \p p0, \p p1, \p p2 is degenerate, bOK returns false.
*/
CLASS_DECLSPEC
Vector2 getNormalVector(	const Point2& p0,
							const Point2& p1,
							const Point2& p2,
							bool& bOK);
#endif



/** \brief Get directed edge
* The directed edges of \p vT are returned \p vDirectedEdgesOut.
* Directed means that each edge (a,b) with two adjacent triangles
* in vT is returned twice, as edge(a,b) and edge(b,a).
*/
CLASS_DECLSPEC
void getDirectedEdges(std::vector<Triangle2*>& vT,std::vector<Edge2>& vDirectedEdgesOut);

/** \brief Get undirected edges
 *
* A unique set of edges of \p vT is returned.
*/
CLASS_DECLSPEC
void getUndirectedEdges(std::vector<Triangle2*>& vT,std::vector<Edge2>& vUndirectedEdgesOut);


/** \brief Fill a hole in a 3D mesh with triangles (deprecated)
 *
 * This function was experimental and is now deprecated because 3D
 * point cloud meshing has been moved to the WOF library.
 *
 * @param [in] vPolygonSegments contains the segments of a closed,
 * simple input polygon along with normal vectors. The segments are
 * counterclockwise oriented and ordered with respect to the surface
 * to be created. Check twice, the orientation is very important.
 * The normal vectors point in the direction of the thought surface
 * at the segment i.e., if a hole is filled, the normal vector of
 * an adjecent triangle is taken but if a T-joint is filled the
 * normal vector should be the average normal of the two triangles
 * at the edge.
 * @param [in] bWithRefine specifies if additional vertices shall
 * be created. (bWithRefine=true is experimental, don't use currently)
 * @param [in] bVerbose specifies if warnings shall be printed to stdout
 * @param [out] vCornersOut contains the created fill triangles, 3
 * corners per triangle, counterclockwise oriented.
 */
CLASS_DECLSPEC
bool fillHole(	std::vector<std::pair<Segment2,Vector2> > vPolygonSegments,
				bool bWithRefine,
				bool bVerbose,
				std::vector<Point2>& vCornersOut
				);




#if GEOM_PSEUDO3D==GEOM_TRUE
/** \brief Fill a hole in a 3D mesh with triangles (deprecated)
 *
 * This function was experimental and is now deprecated because 3D
 * point cloud meshing has been moved to the WOF library.

 * @param [in] vMeshCorners specifies the input mesh, 3 points per
 * triangle in counterclockwise order.
 * @param [in] vPolygonSegments are the edges of the *closed* polygon
 * to be triangulated.
 * @param [in] bWithRefine specifies if additional vertices shall
 * be created (bWithRefine=true is experimental, don't use currently)
 * @param [in] bVerbose specifies if warnings shall be printed to stdout
 * @param [out] vCornersOut contains the created fill triangles, 3
 * corners per triangle, counterclockwise oriented.
 */
CLASS_DECLSPEC
bool fillHole(	std::vector<Point2>& vMeshCorners,
				std::vector<Segment2>& vPolygonSegments,
				bool bWithRefine,
				bool bVerbose,
				std::vector<Point2>& vCornersOut
				);

/** \brief Fill a hole in a 3D mesh with triangles (deprecated)
 *
 * This function was experimental and is now deprecated because 3D
 * point cloud meshing has been moved to the WOF library.
 *
 * @param [in] pMesh
 * @param [in] vPolygonEdges are edges of the polygon to be
 * triangulated. They must form a closed polygon in the mesh.
 * @param [in] bWithRefine specifies if additional vertices shall
 * be created (Note: bWithRefine=true is experimental, don't use currently)
 * @param [in] bVerbose specifies if warnings shall be printed to stdout
 * @param [out] vCornersOut contains the created fill triangles, 3
 * corners per triangle, counterclockwise oriented.
 */
CLASS_DECLSPEC
bool fillHole(	Mesh3* pMesh,
				std::vector<Edge2>& vPolygonEdges,
				bool bWithRefine,
				bool bVerbose,
				std::vector<Point2>& vCornersOut
				);
#endif


/** \brief Create polygons from a set of edges
 *
 *
 * A number of methods in Fade returns an unorganized set of edges that
 * delimit a certain area. But sometimes it is more beneficial to have
 * these edges organized as a set of one or more polygons. This is the
 * purpose of the present method.
 *
 * @param [in] vEdgesIn is a vector of oriented edges
 * @param [out] vvPolygonsOut contains one vector<Edge2> for each polygon
 * found in the input data.
 * @param [out] vRemainingOut is used to return unusable remaining edges
 *
 * The present function adds one vector<Edge2> to \p vvPolygonsOut
 * for each polygon found in \p vEdgesIn. Each such polygon starts
 * with the leftmost vertex (and when two or more
 * vertices share the smallest x-coordiante then the one of them with
 * the smallest y-coordinate is chosen). Edges that do not form a
 * closed polygon are returned in \p vRemainingOut.
 *
 * @note An Edge2 object represents an edge of a triangle. Triangle
 * corners are always counterclockwise (CCW) oriented. Thus outer
 * polygons are CCW-oriented while hole-polygons are CW-oriented,
 * see the figure.
 *
 * \image html edges-to-polygons.png "Polygons created by edgesToPolygons"
 * \image latex edges-to-polygons.eps "Polygons created by edgesToPolygons" width=12cm
 *
 */
CLASS_DECLSPEC
void edgesToPolygons(
	std::vector<Edge2>& vEdgesIn,
	std::vector<std::vector<Edge2> >& vvPolygonsOut,
	std::vector<Edge2>& vRemainingOut
	);

/** \brief Get Borders
 *
 * Computes the border of the triangles in \p vT. The border consists
 * of all edges having only one adjacent triangle in vT.
 *
 * \param [in] vT are the input triangles
 * \param [out] vBorderSegmentsOut is used to return all border segments
*/
CLASS_DECLSPEC
void getBorders(const std::vector<Triangle2*>& vT,std::vector<Segment2>& vBorderSegmentsOut);
/** \brief Sort a vector of Segments
 *
 * The segments in vRing are reoriented and sorted such that subsequent
 * segments join at the endpoints.
*/
CLASS_DECLSPEC
bool sortRing(std::vector<Segment2>& vRing);

/** \brief Sort a vector of Segments
 *
 * The segments in vRing are reoriented and sorted such that the
 * resulting polygon is counterclockwise oriented and subsequent
 * segments join at the endpoints.
*/
CLASS_DECLSPEC
bool sortRingCCW(std::vector<Segment2>& vRing);


/** \brief Get the orientation of three points
 *
 * This function returns the \e exact orientation of the points \p p0, \p p1, \p p2
 * Possible values are \n
 * ORIENTATION2_COLLINEAR if \p p0, \p p1, \p p2 are located on a line, \n
 * ORIENTATION2_CCW if \p p0, \p p1, \p p2 are counterclockwise oriented \n
 * ORIENTATION2_CW if \p p0, \p p1, \p p2 are clockwise oriented \n
 *
 * Not thread-safe but a bit faster than the thread-safe version
*/

FUNC_DECLSPEC
Orientation2 getOrientation2(const Point2* p0,const Point2* p1,const Point2* p2);
/** \brief Get Orientation2 (MT)
 *
 * \see getOrientation2(const Point2* p0,const Point2* p1,const Point2* p2)
 *
 * This version is thread-safe.
*/

FUNC_DECLSPEC
Orientation2 getOrientation2_mt(const Point2* p0,const Point2* p1,const Point2* p2);

/// @private
CLASS_DECLSPEC
const char* getString(const Orientation2 ori);

/** @}*/


/** \defgroup codeInfo Version Information
 *  @{
 */

/** \brief Get the Fade2D version string
*/
CLASS_DECLSPEC
const char* getFade2DVersion();
/** \brief Get the major version number
*/
FUNC_DECLSPEC
int getMajorVersionNumber();
/** \brief Get the minor version number
*/
FUNC_DECLSPEC
int getMinorVersionNumber();
/** \brief Get the revision version number
*/
FUNC_DECLSPEC
int getRevisionNumber();
/** \brief Check if a RELEASE or a DEBUG version is used.
*/
FUNC_DECLSPEC
bool isRelease();


/// @private
FUNC_DECLSPEC
void setLic(
	const char* l1,
	const char* l2,
	const char* dt,
	const char* s1,
	const char* s2_
	);
/// @private
class Lic;
/// @private
Lic* getLic();

/** @}*/








/** \defgroup fileIO File I/O
 *  @{
 */


//////////////////////////
// READ AND WRITE, ASCII
//////////////////////////

/** \brief Write points to an ASCII file
 *
 * Writes points to an ASCII file,
 * \if SECTION_FADE25D
 * three coordinates (x y z) per line,
 * \else
 * two coordinates (x y) per line,
 * \endif
 * whitespace separated.
 *
 * \note Data exchange through ASCII files is easy and convenient but
 * floating point coordinates are not necessarily exact when represented
 * as decimal numbers. If the tiny rounding errors can't be accepted in
 * your setting you are advised to write binary files, (use
 * writePointsBIN() )
 *
*/
FUNC_DECLSPEC
bool writePointsASCII(const char* filename,const std::vector<Point2*>& vPointsIn);

/** \brief Write points to an ASCII file
 *
 * Write points to an ASCII file
 *
 * \see readPointsASCII()
*/
CLASS_DECLSPEC
bool writePointsASCII(const char* filename,const std::vector<Point2>& vPointsIn);





/** \brief Read (x y) points
 *
 * Reads points from an ASCII file. Expected file format: Two
 * coordinates (x y) per line, whitespace separated.
 *
 * \cond SECTION_FADE25D
 * The z coordinate is set to 0.
 * \endcond
*/
FUNC_DECLSPEC
bool readXY(const char* filename,std::vector<Point2>& vPointsOut);

#if GEOM_PSEUDO3D==GEOM_TRUE
// ONLY 2.5D
/** \brief Read (x y z) points
 *
 * Reads points from an ASCII file. Expected file format: Three
 * coordinates (x y z) per line, whitespace separated.
*/
FUNC_DECLSPEC
bool readXYZ(const char* filename,std::vector<Point2>& vPointsOut);
#endif


//////////////////////////
// READ AND WRITE, BINARY
//////////////////////////



/** \brief Write points to a binary file
 *
 * File format:\n
 * \if SECTION_FADE25D
 * int filetype (30)\n
 * \else
 * int filetype (20)\n
 * \endif
 * size_t numPoints (\p vPointsIn.size())\n
 * double x0\n
 * double y0\n
 * double z0\n
 * ...\n
 * double xn\n
 * double yn\n
 * double zn\n
 *
 * @note Since version 1.64 the binary file format written by 32-bit machines
 * is identical with the file format of x64 machines i.e., the numPoints value
 * is always 8 bytes, not 4. This change affects only 32-bit programs.
*/
CLASS_DECLSPEC
bool writePointsBIN(const char* filename,std::vector<Point2>& vPointsIn);

/** \brief Write points to a binary file
 *
 * Writes points to a binary file
 *
 * \see readPointsBIN()
*/
CLASS_DECLSPEC
bool writePointsBIN(const char* filename,std::vector<Point2*>& vPointsIn);

/** \brief Read points from a binary file
 *
 * Reads points from a binary file.
 *
 * \see writePointsBIN()
*/
CLASS_DECLSPEC
bool readPointsBIN(const char* filename, std::vector<Point2>& vPointsIn);

/** \brief Write segments to a binary file
 *
 * Binary file format:\n
 * \if SECTION_FADE25D
 * int filetype (31) \n
 * \else
 * int filetype (21) \n
 * \endif
 * size_t numSegments (\p vSegmentsIn.size())  \n
 * double x0_source \n
 * double y0_source \n
 * \if SECTION_FADE25D
 * double z0_source \n
 * \endif
 * double x0_target \n
 * double y0_target \n
 * \if SECTION_FADE25D
 * double z0_target \n
 * \endif
 * ... \n
 * double xn_source \n
 * double yn_source \n
 * \if SECTION_FADE25D
 * double zn_source \n
 * \endif
 * double xn_target \n
 * double yn_target \n
 * \if SECTION_FADE25D
 * double zn_target \n
 * \endif
 *
 * @note Since version 1.64 the binary file format written by 32-bit machines
 * is identical with the file format of x64 machines i.e., the numSegments value
 * is always 8 bytes, not 4. This change affects only 32-bit programs.
 *
 * \see readSegmentsBIN()
*/
CLASS_DECLSPEC
bool writeSegmentsBIN(const char* filename,std::vector<Segment2>& vSegmentsIn);

/** \brief Read segments from a binary file
 *
 * Reads segments from a binary file of type 21 or 31
 *
 * \see writeSegmentsBIN()
*/
CLASS_DECLSPEC
bool readSegmentsBIN(const char* filename,std::vector<Segment2>& vSegmentsOut);






/** @}*/
} // NAMESPACE
