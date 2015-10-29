#include <iostream>
#include <dart/dynamics/dynamics.h>
#include <dart/utils/urdf/DartLoader.h>
#include <r3/util/CatkinResourceRetriever.h>

static std::vector<std::string> const dof_names {
  "/right/j1",
  "/right/j2",
  "/right/j3",
  "/right/j4",
  "/right/j5",
  "/right/j6",
  "/right/j7"
};

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cerr << "Incorrect number of arguments.\n"
                 "Usage: ./inversedynamics_test <urdf_uri>"
              << std::endl;
    return 1;
  }

  std::string const input_urdf_uri(argv[1]);

  dart::common::Uri urdf_uri;
  if (!urdf_uri.fromString(input_urdf_uri)) {
    std::cerr << "Failed parsing URI '" << input_urdf_uri << "'." << std::endl;
    return 1;
  }

  dart::utils::DartLoader urdf_loader;
  auto const resource_retriever
    = std::make_shared<r3::util::CatkinResourceRetriever>();
  dart::dynamics::SkeletonPtr skeleton = urdf_loader.parseSkeleton(
    urdf_uri, resource_retriever);
  if (!skeleton) {
    std::cerr << "Failed loading URDF as a DART Skeleton." << std::endl;
    return 1;
  }

  dart::dynamics::GroupPtr const controlled_skeleton
    = dart::dynamics::Group::create();

  for (const std::string &dof_name : dof_names ) {
    dart::dynamics::DegreeOfFreedom *const dof = skeleton->getDof(dof_name);
    if (dof) {
      controlled_skeleton->addDof(dof, true);
    } else {
      std::cerr << "There is no DOF named '" << dof_name << "'." << std::endl;
      return 1;
    }
  }

  std::istream &input_file = std::cin;

  size_t const num_dof = controlled_skeleton->getNumDofs();
  controlled_skeleton->setVelocities(Eigen::VectorXd::Zero(num_dof));
  controlled_skeleton->setAccelerations(Eigen::VectorXd::Zero(num_dof));

  for (size_t iline = 0; input_file; ++iline) {
    // Read joint positions and torques from the input file.
    Eigen::VectorXd positions(num_dof);
    for (size_t i = 0; i < num_dof && input_file; ++i)
      input_file >> positions[i];

    Eigen::VectorXd torques_actual(num_dof);
    for (size_t i = 0; i < num_dof && input_file; ++i)
      input_file >> torques_actual[i];

    if (!input_file) {
      std::cerr << "Failed reading line " << (iline + 1) << "." << std::endl;
      break;
    }

    // Compute the inverse dynamics solution.
    skeleton->computeInverseDynamics();
    Eigen::VectorXd const torques_computed = controlled_skeleton->getForces();
    Eigen::VectorXd const torques_error = torques_actual - torques_computed;

    for (size_t i = 0; i < num_dof; ++i)
      std::cout << torques_error[i] << " ";
  }
  return 0;
}
