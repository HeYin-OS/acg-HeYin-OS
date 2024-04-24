#include <filesystem>
// #include <experimental/filesystem> // uncomment here if the <filesystem> cannot be included above
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Eigen/Core"
//
#include "parse_svg.h"

/***
 * signed area of a triangle connecting points (p0, p1, p2) in counter-clockwise order.
 * @param p0 1st point xy-coordinate
 * @param p1 2nd point xy-coordinate
 * @param p2 3rd point xy-coordinate
 * @return signed area (float)
 */
float area(
    const Eigen::Vector2f &p0,
    const Eigen::Vector2f &p1,
    const Eigen::Vector2f &p2) {
  const auto v01 = p1 - p0;
  const auto v02 = p2 - p0;
  // return 0.5f * (v01[0] * v02[1] - v01[1] * v02[0]); // right handed coordinate
  return 0.5f * (v01[1] * v02[0] - v01[0] * v02[1]); // left-handed coordinate (because pixel y-coordinate is going down)
}


/***
 * compute number of intersection of a ray against a line segment
 * @param org ray origin
 * @param dir ray direction (unit normal)
 * @param ps one of the two end points
 * @param pe the other end point
 * @return number of intersection
 */
int number_of_intersection_ray_against_edge(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pe) {
  auto a = area(org, org + dir, ps);
  auto b = area(org, pe, org + dir);
  auto c = area(org, ps, pe);
  auto d = area(dir+ps, ps, pe);
  if (a * b > 0.f && d * c < 0.f) { return 1; }
  return 0;
  // the following code was a bug
  //auto d = area(org + dir, ps, pe);
  //if (a * b > 0.f && d * c > 0.f && fabs(d) > fabs(c)) { return 1; }
}

/***
 *
 * @param org ray origin
 * @param dir ray direction (unit vector)
 * @param ps one of the two end points
 * @param pc control point
 * @param pe the other end point
 * @return the number of intersections
 */
int number_of_intersection_ray_against_quadratic_bezier(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pc,
    const Eigen::Vector2f &pe) {
    // comment out below to do the assignment
    //if (((pc.x() == ps.x()) && (pc.x() == pe.x())) || ((pc.y() == ps.y()) && (pc.y() == pe.y())))
     //   return number_of_intersection_ray_against_edge(org, dir, ps, pe);
    // write some code below to find the intersection between ray and the quadratic
    auto w = Eigen::Vector2f(-dir.y(), dir.x());
    // a,b,c of quadratic equation in two variables (the coefficients have already been combined on my scratch paper)
    auto a = (pe.x() - 2 * pc.x() + ps.x()) * w.x() + (pe.y() - 2 * pc.y() + ps.y()) * w.y();
    auto b = 2 * ((pc.x() - ps.x()) * w.x() + (pc.y() - ps.y()) * w.y());
    auto c = (ps.x() - org.x()) * w.x() + (ps.y() - org.y()) * w.y();
    // solve quadratic equation in two variables
    auto delta = pow(b, 2) - 4 * a * c;
    //no real root
    if (delta < 0.f) return 0;
    auto sqrt_of_delta = sqrt(delta);
    //get t1 and t2
    auto t1 = (-b + sqrt_of_delta) / 2 / a;
    auto t2 = (-b - sqrt_of_delta) / 2 / a;
    if ((t1 < 0.f || t1 > 1.f) && (t2 < 0.f || t2 > 1.f)) return 0;
    auto p_x_1 = pow(1 - t1, 2) * ps.x() + 2 * (1 - t1) * t1 * pc.x() + pow(t1, 2) * pe.x();
    auto p_x_2 = pow(1 - t2, 2) * ps.x() + 2 * (1 - t2) * t2 * pc.x() + pow(t2, 2) * pe.x();
    auto div_1_x = (p_x_1 - org.x()) / dir.x();
    auto div_2_x = (p_x_2 - org.x()) / dir.x();
    if ((t1 > 0.f && t1 < 1.f) && (t2 < 0.f || t2 > 1.f)) {
        if (div_1_x > 0) return 1;
        else return 0;
    } else if ((t1 < 0.f || t1 > 1.f) && (t2 > 0.f && t2 < 1.f)) {
        if (div_2_x > 0) return 1;
        else return 0;
    } else {
        if (div_1_x > 0 && div_2_x > 0) return 2;
        else if (div_1_x > 0 || div_2_x > 0) return 1;
        else return 0;
    }
}

int main() {
  const auto input_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / ".." / "asset" / "r.svg";
  const auto [width, height, shape] = acg::svg_get_image_size_and_shape(input_file_path);
  if (width == 0) { // something went wrong in loading the function
    std::cout << "file open failure" << std::endl;
    abort();
  }
  const std::vector<std::string> outline_path = acg::svg_outline_path_from_shape(shape);
  const std::vector<std::vector<acg::Edge>> loops = acg::svg_loops_from_outline_path(outline_path);
  //
  std::vector<unsigned char> img_data(width * height, 255); // grayscale image initialized white
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto org = Eigen::Vector2f(iw + 0.5, ih + 0.5); // pixel center
      const auto dir = Eigen::Vector2f(60., 20.); // search direction
      int count_cross = 0;
      for (const auto &loop: loops) { // loop over loop (letter R have internal/external loops)
        for (const auto &edge: loop) { // loop over edge in the loop
          if (edge.is_bezier) { // in case the edge is a quadratic Bézier
            count_cross += number_of_intersection_ray_against_quadratic_bezier(
                org, dir,
                edge.ps, edge.pc, edge.pe);
          } else { // in case the edge is a line segment
            count_cross += number_of_intersection_ray_against_edge(
                org, dir,
                edge.ps, edge.pe);
          }
        }
      }
      if (count_cross % 2 == 1) { // Jordan's curve theory
        img_data[ih * width + iw] = 0; // paint black if it is inside
      }
    }
  }
  const auto output_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png";
  stbi_write_png(output_file_path.string().c_str(), width, height, 1, img_data.data(), width);
}
