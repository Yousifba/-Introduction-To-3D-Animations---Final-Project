// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>
#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>
#include <igl/collapse_edge.h>

#include <igl/circulation.h>
#include <unordered_set>
#include <igl/edge_collapse_is_valid.h>

#include <windows.h>



// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
	namespace opengl
	{
		namespace glfw
		{

			IGL_INLINE void Viewer::init()
			{


			}

			//IGL_INLINE void Viewer::init_plugins()
			//{
			//  // Init all plugins
			//  for (unsigned int i = 0; i<plugins.size(); ++i)
			//  {
			//    plugins[i]->init(this);
			//  }
			//}

			//IGL_INLINE void Viewer::shutdown_plugins()
			//{
			//  for (unsigned int i = 0; i<plugins.size(); ++i)
			//  {
			//    plugins[i]->shutdown();
			//  }
			//}

			IGL_INLINE Viewer::Viewer() :
				data_list(1),
				selected_data_index(0),
				next_data_id(1)
			{
				data_list.front().id = 0;



				// Temporary variables initialization
			   // down = false;
			  //  hack_never_moved = true;
				scroll_position = 0.0f;

				// Per face
				data().set_face_based(false);


#ifndef IGL_VIEWER_VIEWER_QUIET
				const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
				);
				std::cout << usage << std::endl;
#endif
			}

			IGL_INLINE Viewer::~Viewer()
			{
			}

			IGL_INLINE bool Viewer::load_mesh_from_file(
				const std::string& mesh_file_name_string)
			{

				// Create new data slot and set to selected
				if (!(data().F.rows() == 0 && data().V.rows() == 0))
				{
					append_mesh();
				}
				data().clear();

				size_t last_dot = mesh_file_name_string.rfind('.');
				if (last_dot == std::string::npos)
				{
					std::cerr << "Error: No file extension found in " <<
						mesh_file_name_string << std::endl;
					return false;
				}

				std::string extension = mesh_file_name_string.substr(last_dot + 1);

				if (extension == "off" || extension == "OFF")
				{
					Eigen::MatrixXd V;
					Eigen::MatrixXi F;
					if (!igl::readOFF(mesh_file_name_string, V, F))
						return false;
					data().set_mesh(V, F);
				}
				else if (extension == "obj" || extension == "OBJ")
				{
					Eigen::MatrixXd corner_normals;
					Eigen::MatrixXi fNormIndices;

					Eigen::MatrixXd UV_V;
					Eigen::MatrixXi UV_F;
					Eigen::MatrixXd V;
					Eigen::MatrixXi F;

					if (!(
						igl::readOBJ(
							mesh_file_name_string,
							V, UV_V, corner_normals, F, UV_F, fNormIndices)))
					{
						return false;
					}

					data().set_mesh(V, F);
					data().set_uv(UV_V, UV_F);

				}
				else
				{
					// unrecognized file type
					printf("Error: %s is not a recognized file type.\n", extension.c_str());
					return false;
				}

				data().compute_normals();
				data().uniform_colors(Eigen::Vector3d(51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0),
					Eigen::Vector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0),
					Eigen::Vector3d(255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0));

				// Alec: why?
				if (data().V_uv.rows() == 0)
				{
					data().grid_texture();
				}


				//for (unsigned int i = 0; i<plugins.size(); ++i)
				//  if (plugins[i]->post_load())
				//    return true;

				return true;
			}

			IGL_INLINE bool Viewer::save_mesh_to_file(
				const std::string& mesh_file_name_string)
			{
				// first try to load it with a plugin
				//for (unsigned int i = 0; i<plugins.size(); ++i)
				//  if (plugins[i]->save(mesh_file_name_string))
				//    return true;

				size_t last_dot = mesh_file_name_string.rfind('.');
				if (last_dot == std::string::npos)
				{
					// No file type determined
					std::cerr << "Error: No file extension found in " <<
						mesh_file_name_string << std::endl;
					return false;
				}
				std::string extension = mesh_file_name_string.substr(last_dot + 1);
				if (extension == "off" || extension == "OFF")
				{
					return igl::writeOFF(
						mesh_file_name_string, data().V, data().F);
				}
				else if (extension == "obj" || extension == "OBJ")
				{
					Eigen::MatrixXd corner_normals;
					Eigen::MatrixXi fNormIndices;

					Eigen::MatrixXd UV_V;
					Eigen::MatrixXi UV_F;

					return igl::writeOBJ(mesh_file_name_string,
						data().V,
						data().F,
						corner_normals, fNormIndices, UV_V, UV_F);
				}
				else
				{
					// unrecognized file type
					printf("Error: %s is not a recognized file type.\n", extension.c_str());
					return false;
				}
				return true;
			}

			IGL_INLINE bool Viewer::load_scene()
			{
				std::string fname = igl::file_dialog_open();
				if (fname.length() == 0)
					return false;
				return load_scene(fname);
			}

			IGL_INLINE bool Viewer::load_scene(std::string fname)
			{
				// igl::deserialize(core(),"Core",fname.c_str());
				igl::deserialize(data(), "Data", fname.c_str());
				return true;
			}

			IGL_INLINE bool Viewer::save_scene()
			{
				std::string fname = igl::file_dialog_save();
				if (fname.length() == 0)
					return false;
				return save_scene(fname);
			}

			IGL_INLINE bool Viewer::save_scene(std::string fname)
			{
				//igl::serialize(core(),"Core",fname.c_str(),true);
				igl::serialize(data(), "Data", fname.c_str());

				return true;
			}

			IGL_INLINE void Viewer::open_dialog_load_mesh()
			{
				std::string fname = igl::file_dialog_open();

				if (fname.length() == 0)
					return;

				this->load_mesh_from_file(fname.c_str());
			}

			IGL_INLINE void Viewer::open_dialog_save_mesh()
			{
				std::string fname = igl::file_dialog_save();

				if (fname.length() == 0)
					return;

				this->save_mesh_to_file(fname.c_str());
			}

			IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
			{
				assert(!data_list.empty() && "data_list should never be empty");
				int index;
				if (mesh_id == -1)
					index = selected_data_index;
				else
					index = mesh_index(mesh_id);

				assert((index >= 0 && index < data_list.size()) &&
					"selected_data_index or mesh_id should be in bounds");
				return data_list[index];
			}

			IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
			{
				assert(!data_list.empty() && "data_list should never be empty");
				int index;
				if (mesh_id == -1)
					index = selected_data_index;
				else
					index = mesh_index(mesh_id);

				assert((index >= 0 && index < data_list.size()) &&
					"selected_data_index or mesh_id should be in bounds");
				return data_list[index];
			}

			IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
			{
				assert(data_list.size() >= 1);

				data_list.emplace_back();
				selected_data_index = data_list.size() - 1;
				data_list.back().id = next_data_id++;
				//if (visible)
				//    for (int i = 0; i < core_list.size(); i++)
				//        data_list.back().set_visible(true, core_list[i].id);
				//else
				//    data_list.back().is_visible = 0;
				return data_list.back().id;
			}

			IGL_INLINE bool Viewer::erase_mesh(const size_t index)
			{
				assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
				assert(data_list.size() >= 1);
				if (data_list.size() == 1)
				{
					// Cannot remove last mesh
					return false;
				}
				data_list[index].meshgl.free();
				data_list.erase(data_list.begin() + index);
				if (selected_data_index >= index && selected_data_index > 0)
				{
					selected_data_index--;
				}

				return true;
			}

			IGL_INLINE size_t Viewer::mesh_index(const int id) const {
				for (size_t i = 0; i < data_list.size(); ++i)
				{
					if (data_list[i].id == id)
						return i;
				}
				return 0;
			}

			void Viewer::level_handler()
			{
				if (score < cur_level_max_score)
					return;
				else 
				{
					
					finished_objective = true;
					data_list[data_list.size() - 4].set_visible(true, left_view->id);
					data_list[data_list.size() - 5].set_visible(true, right_view->id);
				}
					

				if(finished_objective && finished_level)
				{				
					PlaySound(NULL, NULL, SND_FILENAME | SND_ASYNC);
					PlaySound(TEXT("level_up.wav"), NULL, SND_FILENAME | SND_ASYNC);
					SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 10);
					printf("\nCash: %d$               Lives: %d\n", cash, lives);
					SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 7);
					printf("Great! You Finished The Level:\n\n");
					printf("1.Procced To The Next Level.\n");
					printf("2.Open The Shop.\n");
					printf("3.Exit The Game.\n");
					int input;
					std::cin >> input;
					switch (input)
					{
						case 1:
						{
							load_next_level();
							break;
						}
						case 2:
						{
							int shop_opt = -1;
							while (shop_opt != 4)
							{
								SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 10);
								printf("\nCash: %d$               Lives: %d\n", cash, lives);
								SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 7);
								printf("What'd You Like To Purchase: \n");
								printf("%-50s Price\n", "Item");
								printf("%-50s 5$\n", "1.Crispy Chicken (Increases Snake Length)");
								printf("%-50s 5$\n", "2.A Pair Of Sneakers (Increases Snake Speed)");
								printf("%-50s 10$\n", "3.Extra Life");
								printf("4.Return To Menu\n");
								
								std::cin >> shop_opt;
								switch (shop_opt)
								{
								case 1:
								{
									if (cash < 5)
									{
										SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 4);
										printf("\nYou Don't Have Enough Money!\n");
										SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 7);
									}									
									else
									{
										PlaySound(NULL, NULL, SND_FILENAME | SND_ASYNC);
										PlaySound(TEXT("length_upgrade.wav"), NULL, SND_FILENAME | SND_ASYNC);
										snake_length_upgrade++;
										cash -= 5;
									}
									break;
								}
								case 2:
								{
									if (cash < 5)
									{
										SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 4);
										printf("\nYou Don't Have Enough Money!\n");
										SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 7);
									}						
									else
									{
										PlaySound(NULL, NULL, SND_FILENAME | SND_ASYNC);
										PlaySound(TEXT("speed_upgrade.wav"), NULL, SND_FILENAME | SND_ASYNC);
										snake_speed++;
										snake_speed_upgrade++;
										cash -= 5;
									}
									break;
								}
								case 3:
								{
									if (cash < 10)
									{
										SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 4);
										printf("\nYou Don't Have Enough Money!\n");
										SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 7);
									}
									else
									{
										lives++;
										cash -= 10;
									}
									break;
								}
								case 4:
									break;
								}
							}
							level_handler();
							break;
						}
						case 3:
						{
							printf("Not Yet.\n");
							break;
						}
					}
				}
			}

			void Viewer::load_next_level()
			{
				loading = true;
				finished_level = false;
				finished_objective = false;
				run_ik = false;
				found_obj = false;
				cur_level++;

				for (int i = data_list.size() - 1; i >= arm_length; i--)
				{
					erase_mesh(i);
				}
				kd_trees.clear();
				load_snake();
				if (snake_length_upgrade > 0)
				{
					

					arm_length += 2 * snake_length_upgrade;
					SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 2);
					printf("Snake Length Upgraded To %d!\n", arm_length);
					SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 7);
					Eigen::MatrixXd& V = data().V;
					Eigen::MatrixXi& F = data().F;
					Eigen::Vector3d m = V.colwise().minCoeff();
					Eigen::Vector3d M = V.colwise().maxCoeff();
					link_length = abs(M(1) - m(1));

					for (int i = 0; i < 2 * snake_length_upgrade; i++)
					{
						if (!load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/ycylinder.obj"))
							std::cout << "Failed To Upgrade Snake Length" << std::endl;
						data_list[selected_data_index].getTrans().pretranslate(Eigen::Vector3f(0, link_length * (selected_data_index), 0));
						parent_axis_coordinates[selected_data_index] = Eigen::Vector4f(0, 0.8 + link_length * (selected_data_index), 0, 1);
						parent_axis_rotation[selected_data_index] = Eigen::Matrix4f::Identity();
						data().uniform_colors_index(1);
						data().set_face_based(false);
					}
					snake_length_upgrade = 0;
					save_snake();
				}
				if (snake_speed_upgrade > 0)
				{
					SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 2);
					printf("Snake Speed Upgraded To %dx!\n", snake_speed);
					SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 7);
					snake_speed_upgrade = 0;
				}

				switch (cur_level % 3)
				{
				case 2:							
					cur_level_max_score = cur_level * 50;
					score = 0;
					load_balls(cur_level);
					break;
				default:
					cur_level_max_score = cur_level * 50;
					score = 0;
					load_balls(cur_level);
					break;
				}
				load_environment();
				build_kd_trees();

				for (auto& data : data_list)
				{
					if (data.id != left_arrow && data.id != right_arrow)
					{
						data.set_visible(true, left_view->id);
						data.set_visible(true, right_view->id);
						data.copy_options(*left_view, *right_view);
					}
					else
					{
						data.set_visible(false, left_view->id);
						data.set_visible(false, right_view->id);
						data.copy_options(*left_view, *right_view);
					}
				}

				selected_data_index = 0;

				loaded_new_level = true;

				loading = false;
			}

			void Viewer::load_balls(int n)
			{
				for (int i = 0; i < n; i++)
				{
					load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/sphere.obj");
					movement m1;
					double x = (double)rand() / RAND_MAX;
					x = -0.5f + x * (1.0f);
					double y = (double)rand() / RAND_MAX;
					y = -0.5f + y * (1.0f);

					m1.velocity = Eigen::Vector3f(x, y, 0) / 30;
					m1.elasticity = 0.80f;
					if (i >= movement_data.size())
						movement_data.push_back(m1);
					else
						movement_data.at(i) = m1;

					data().Move(Eigen::Vector4f(1.5f * pow(-1.85f, i) + pow(-1, i), 1.5f * pow(-1.85f, i) + pow(-1, i), 0, 1));
					data_list[i + arm_length].getTrans().pretranslate(Eigen::Vector3f(0, 0, 0.9f));
					data().set_face_based(false);
				}
			}

			void Viewer::load_environment()
			{
				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/grass.obj");
				data().uniform_colors_index(4);
				data().shininess = 5.0f;
				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/Wall.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(0, 57, 0));
				data().uniform_colors_index(5);
				data().shininess = 100.0f;
				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/Pyramid.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(0, 52.0f, 0));

				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/ArrowRight.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(-15, 35.0f, 0));
				data_list[data_list.size() - 1].set_visible(false, left_view->id);
				data_list[data_list.size() - 1].set_visible(false, right_view->id);
				right_arrow = data_list[data_list.size() - 1].id;

				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/ArrowLeft.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(15, 35.0f, 0));
				data_list[data_list.size() - 1].set_visible(false, left_view->id);
				data_list[data_list.size() - 1].set_visible(false, right_view->id);
				left_arrow = data_list[data_list.size() - 1].id;

				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/Wall.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(0, -57, 0));
				data().uniform_colors_index(5);
				data().shininess = 100.0f;
				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/WallSides.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(57, 0, 0));
				data().uniform_colors_index(5);
				data().shininess = 100.0f;
				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/WallSides.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(-57, 0, 0));
				data().uniform_colors_index(5);
				data().shininess = 100.0f;
			}

			void Viewer::save_snake()
			{
				saved_snake_data.clear();
				for (int i = 0; i < arm_length; i++)
				{
					saved_parent_axis_coordinates[i] = parent_axis_coordinates[i];
					saved_parent_axis_rotation[i] = parent_axis_rotation[i];
					saved_snake_data.push_back(data_list[i]);
				}
				saved_arm_root = arm_root;
				saved_arm_root_rotation = arm_root_rotation;
			}

			void Viewer::load_snake()
			{
				for (int i = 0; i < arm_length; i++)
				{
					parent_axis_coordinates[i] = saved_parent_axis_coordinates[i];
					parent_axis_rotation[i] = saved_parent_axis_rotation[i];
					data_list.at(i) = saved_snake_data.at(i);
				}
				arm_root = saved_arm_root;
				arm_root_rotation = saved_arm_root_rotation;
				arm_scale = 1.0f;
			}


			void Viewer::collision_handler()
			{
				if (loading)
					return;
				for (int i = 0; i < arm_length; i++)
				{
					if (finished_objective)
					{
						bool collision = check_for_collision(kd_trees.at(i), kd_trees.at(data_list.size() - 6), i, data_list.size() - 6);
						if (collision)
						{
							finished_level = true;
							return;
						}
					}
					for (int j = arm_length; j < data_list.size() - 8; j++)
				{
					bool collision = false;
					if (true/* || in % 5 == 0*/)
					{
						collision = check_for_collision(kd_trees.at(i), kd_trees.at(j), i, j);
						if (collision)
						{
							printf("There has been a collision! (%d, %d)\n", i, j);
							PlaySound(NULL, NULL, SND_FILENAME | SND_ASYNC);
							PlaySound(TEXT("bounce.wav"), NULL, SND_FILENAME | SND_ASYNC);
							erase_mesh(j);
							movement_data.erase(movement_data.begin() + (j - arm_length));
							update = false;
							score += 50;
							cash += 5;
							selected_data_index = 0;
						}
					}
				}
				}

				in++;
				
			}

			bool Viewer::get_separating_axis(Eigen::Vector3f& delta, Eigen::Vector3f& plane, OBB& box1, OBB& box2)
			{
				return (fabs(delta.dot(plane)) >
					fabs((box1.axisX * box1.halfSizes.x()).dot(plane)) +
					fabs((box1.axisY * box1.halfSizes.y()).dot(plane)) +
					fabs((box1.axisZ * box1.halfSizes.z()).dot(plane)) +
					fabs((box2.axisX * box2.halfSizes.x()).dot(plane)) +
					fabs((box2.axisY * box2.halfSizes.y()).dot(plane)) +
					fabs((box2.axisZ * box2.halfSizes.z()).dot(plane)));
			}

			// checks all 15 axis
			bool Viewer::get_collision(OBB& box1, OBB& box2)
			{
				Eigen::Vector3f delta = box2.position - box1.position;

				return !(get_separating_axis(delta, box1.axisX, box1, box2) ||
					get_separating_axis(delta, box1.axisY, box1, box2) ||
					get_separating_axis(delta, box1.axisZ, box1, box2) ||
					get_separating_axis(delta, box2.axisX, box1, box2) ||
					get_separating_axis(delta, box2.axisY, box1, box2) ||
					get_separating_axis(delta, box2.axisZ, box1, box2) ||
					get_separating_axis(delta, box1.axisX.cross(box2.axisX), box1, box2) ||
					get_separating_axis(delta, box1.axisX.cross(box2.axisY), box1, box2) ||
					get_separating_axis(delta, box1.axisX.cross(box2.axisZ), box1, box2) ||
					get_separating_axis(delta, box1.axisY.cross(box2.axisX), box1, box2) ||
					get_separating_axis(delta, box1.axisY.cross(box2.axisY), box1, box2) ||
					get_separating_axis(delta, box1.axisY.cross(box2.axisZ), box1, box2) ||
					get_separating_axis(delta, box1.axisZ.cross(box2.axisX), box1, box2) ||
					get_separating_axis(delta, box1.axisZ.cross(box2.axisY), box1, box2) ||
					get_separating_axis(delta, box1.axisZ.cross(box2.axisZ), box1, box2));
			}


			bool Viewer::check_for_collision(AABB<Eigen::MatrixXd, 3>& aabb_0, AABB<Eigen::MatrixXd, 3>& aabb_1, int i, int j)
			{
				using namespace Eigen;
				AlignedBox3f b0 = AlignedBox3f(aabb_0.m_box);
				AlignedBox3f b1 = AlignedBox3f(aabb_1.m_box);
				Matrix4f t0;
				Matrix4f t1;

				t0 <<
					scales.at(i), 0, 0, data_list[i].getTrans().translation().x(),
					0, scales.at(i), 0, data_list[i].getTrans().translation().y(),
					0, 0, scales.at(i), data_list[i].getTrans().translation().z(),
					0, 0, 0, 1;

				t1 <<
					scales.at(j), 0, 0, data_list[j].getTrans().translation().x(),
					0, scales.at(j), 0, data_list[j].getTrans().translation().y(),
					0, 0, scales.at(j), data_list[j].getTrans().translation().z(),
					0, 0, 0, 1;

				Vector4f b0_min(b0.m_min.x(), b0.m_min.y(), b0.m_min.z(), 1);
				Vector4f b1_min(b1.m_min.x(), b1.m_min.y(), b1.m_min.z(), 1);
				Vector4f b0_max(b0.m_max.x(), b0.m_max.y(), b0.m_max.z(), 1);
				Vector4f b1_max(b1.m_max.x(), b1.m_max.y(), b1.m_max.z(), 1);
				b0_min = t0 * b0_min;
				b1_min = t1 * b1_min;
				b0_max = t0 * b0_max;
				b1_max = t1 * b1_max;

				/*
				b0.m_min.x() = fmin(b0_min.x(), b0_max.x()); b0.m_min.y() = fmin(b0_min.y(), b0_max.y()); b0.m_min.z() = fmin(b0_min.z(), b0_max.z());
				b1.m_min.x() = fmin(b1_min.x(), b1_max.x()); b1.m_min.y() = fmin(b1_min.y(), b1_max.y()); b1.m_min.z() = fmin(b1_min.z(), b1_max.z());
				b0.m_max.x() = fmax(b0_min.x(), b0_max.x()); b0.m_max.y() = fmax(b0_min.y(), b0_max.y()); b0.m_max.z() = fmax(b0_min.z(), b0_max.z());
				b1.m_max.x() = fmax(b1_min.x(), b1_max.x()); b1.m_max.y() = fmax(b1_min.y(), b1_max.y()); b1.m_max.z() = fmax(b1_min.z(), b1_max.z());*/
				b0.m_min.x() = b0_min.x(); b0.m_min.y() = b0_min.y(); b0.m_min.z() = b0_min.z();
				b1.m_min.x() = b1_min.x(); b1.m_min.y() = b1_min.y(); b1.m_min.z() = b1_min.z();
				b0.m_max.x() = b0_max.x(); b0.m_max.y() = b0_max.y(); b0.m_max.z() = b0_max.z();
				b1.m_max.x() = b1_max.x(); b1.m_max.y() = b1_max.y(); b1.m_max.z() = b1_max.z();

				Vector3f right(1, 0, 0), up(0, 1, 0), forward(0, 0, 1);
				Matrix3f rotation_0 = data_list[i].getTrans().rotation();
				Matrix3f rotation_1 = data_list[j].getTrans().rotation();

				OBB obb_0, obb_1;
				obb_0.position = Vector3f(b0.center().x(), b0.center().y(), b0.center().z());
				obb_0.axisX = (rotation_0 * right); obb_0.axisY = (rotation_0 * up); obb_0.axisZ = (rotation_0 * forward);
				obb_0.halfSizes = Vector3f(b0.sizes().x() / 2, b0.sizes().y() / 2, b0.sizes().z() / 2);

				obb_1.position = Vector3f(b1.center().x(), b1.center().y(), b1.center().z());
				obb_1.axisX = (rotation_1 * right); obb_1.axisY = (rotation_1 * up); obb_1.axisZ = (rotation_1 * forward);
				obb_1.halfSizes = Vector3f(b1.sizes().x() / 2, b1.sizes().y() / 2, b1.sizes().z() / 2);

				bool collision = false;

				/*Vector3f obb_0_v[8] = {
					b0.corner(b0.BottomLeftCeil),
					b0.corner(b0.BottomLeftFloor),
					b0.corner(b0.BottomRightCeil),
					b0.corner(b0.BottomRightFloor),
					b0.corner(b0.TopLeftCeil),
					b0.corner(b0.TopLeftFloor),
					b0.corner(b0.TopRightCeil),
					b0.corner(b0.TopRightFloor)
					};
				Vector3f obb_1_v[8] = {
					b1.corner(b1.BottomLeftCeil),
					b1.corner(b1.BottomLeftFloor),
					b1.corner(b1.BottomRightCeil),
					b1.corner(b1.BottomRightFloor),
					b1.corner(b1.TopLeftCeil),
					b1.corner(b1.TopLeftFloor),
					b1.corner(b1.TopRightCeil),
					b1.corner(b1.TopRightFloor)
				};
				collision = !(
					Separated(obb_0_v, obb_1_v,obb_0.axisX) ||
					Separated(obb_0_v, obb_1_v,obb_0.axisY) ||
					Separated(obb_0_v, obb_1_v,obb_0.axisZ) ||
					Separated(obb_0_v, obb_1_v,obb_1.axisX) ||
					Separated(obb_0_v, obb_1_v,obb_1.axisY) ||
					Separated(obb_0_v, obb_1_v,obb_1.axisZ) ||
					Separated(obb_0_v, obb_1_v,obb_0.axisX.cross(obb_1.axisX)) ||
					Separated(obb_0_v, obb_1_v,obb_0.axisX.cross(obb_1.axisY)) ||
					Separated(obb_0_v, obb_1_v,obb_0.axisX.cross(obb_1.axisZ)) ||
					Separated(obb_0_v, obb_1_v,obb_0.axisY.cross(obb_1.axisX)) ||
					Separated(obb_0_v, obb_1_v,obb_0.axisY.cross(obb_1.axisY)) ||
					Separated(obb_0_v, obb_1_v,obb_0.axisY.cross(obb_1.axisZ)) ||
					Separated(obb_0_v, obb_1_v,obb_0.axisZ.cross(obb_1.axisX)) ||
					Separated(obb_0_v, obb_1_v,obb_0.axisZ.cross(obb_1.axisY)) ||
					Separated(obb_0_v, obb_1_v,obb_0.axisZ.cross(obb_1.axisZ)));*/

				collision = get_collision(obb_0, obb_1);
				if (collision)
				{
					if (aabb_0.is_leaf() || aabb_1.is_leaf())
					{
						/*std::cout << obb_0.Pos << std::endl;
						std::cout << aabb_0.m_box.center() << std::endl;*/
						return collision;
					}
					/*else if (aabb_0.is_leaf())
					{
						return check_for_collision(aabb_0, *aabb_1.m_left) ||
							check_for_collision(aabb_0, *aabb_1.m_right);
					}
					else if (aabb_1.is_leaf())
					{
						return check_for_collision(*aabb_0.m_left, aabb_1) ||
							check_for_collision(*aabb_0.m_left, aabb_1);
					}*/
					else
					{
						return check_for_collision(*aabb_0.m_left, *aabb_1.m_left, i, j) ||
							check_for_collision(*aabb_0.m_right, *aabb_1.m_right, i, j) ||
							check_for_collision(*aabb_0.m_right, *aabb_1.m_left, i, j) ||
							check_for_collision(*aabb_0.m_left, *aabb_1.m_right, i, j);
					}
				}
				return collision;
			}


			void Viewer::build_kd_trees()
			{
				using namespace Eigen;
				for (int i = 0; i < data_list.size() - 1; i++)
				{
					igl::AABB<MatrixXd, 3> tree;
					tree.init(data_list[i].V, data_list[i].F);
					kd_trees.push_back(tree);
					scales.push_back(1);
					//in = 0;
				}
			}

			void Viewer::draw_bounding_boxes()
			{
				using namespace Eigen;

				if (bounding_boxes_visible)
				{
					for (int i = 0; i < data_list.size(); i++)
					{
						MatrixXd V = data_list[i].V;
						MatrixXi F = data_list[i].F;
						data_list[i].clear();
						data_list[i].set_mesh(V, F);
						if (i < arm_length)
							data_list[i].uniform_colors_index(1);
					}
					return;
				}
				
				for (int i = 0; i < data_list.size(); i++)
				{
					MatrixXd V = data_list[i].V;
					MatrixXi F = data_list[i].F;

					// Find the bounding box
					Vector3d m = V.colwise().minCoeff();
					Vector3d M = V.colwise().maxCoeff();

					// Corners of the bounding box
					Eigen::MatrixXd V_box(8, 3);
					V_box <<
						m(0), m(1), m(2),
						M(0), m(1), m(2),
						M(0), M(1), m(2),
						m(0), M(1), m(2),
						m(0), m(1), M(2),
						M(0), m(1), M(2),
						M(0), M(1), M(2),
						m(0), M(1), M(2);

					// Edges of the bounding box
					Eigen::MatrixXi E_box(12, 2);
					E_box <<
						0, 1,
						1, 2,
						2, 3,
						3, 0,
						4, 5,
						5, 6,
						6, 7,
						7, 4,
						0, 4,
						1, 5,
						2, 6,
						7, 3;

					data_list[i].clear();
					data_list[i].set_mesh(V, F);
					if (i < arm_length)
						data_list[i].uniform_colors_index(1);
					// Plot the corners of the bounding box as points
					data_list[i].add_points(V_box, Eigen::RowVector3d(1, 0, 0));

					// Plot the edges of the bounding box
					for (unsigned j = 0;j < E_box.rows(); ++j)
						data_list[i].add_edges
						(
							V_box.row(E_box(j, 0)),
							V_box.row(E_box(j, 1)),
							(i < arm_length ? Eigen::RowVector3d(0.2f, 0.8f, 0.6f) : Eigen::RowVector3d(0, 1, 0))
						);
				}
			}


			void Viewer::update_pos(double deltaTime)
			{
				if (loading)
					return;
				for (int i = 0; i < data_list.size() - arm_length - 8; i++)
				{
					if (data_list[i + arm_length].getTrans().translation().x() > 52.5f)
						movement_data[i].velocity.x() *= -1;
					else if (data_list[i + arm_length].getTrans().translation().x() < -52.5f)
						movement_data[i].velocity.x() *= -1;

					if (data_list[i + arm_length].getTrans().translation().y() > 52.5f)
						movement_data[i].velocity.y() *= -1;
					else if (data_list[i + arm_length].getTrans().translation().y() < -52.5f)
						movement_data[i].velocity.y() *= -1;

						
					else if (data_list[i + arm_length].getTrans().translation().z() < 0.2f)
						movement_data[i].velocity.z() *= -1;

					if (fabs(movement_data[i].velocity.z()) > 0.001f)
					{
						Eigen::Vector3f translation_3 = deltaTime * movement_data[i].velocity / 5.0f;
						data_list[i + arm_length].MyTranslate(translation_3, CAMERA_AXIS);
					}
					
					//data_list[i + arm_length].getTrans().pretranslate(translation_3);
				}	
			}

			int Viewer::sys_init(int n)
			{
				cur_level_max_score = 50 * n;
				load_meshs(n);
				int saved_index = selected_data_index;
				for (int i = 0; i < data_list.size(); i++)
				{
					selected_data_index = i;
					if(i < arm_length)
					data().uniform_colors_index(1);
					load_meshs_ik();
					if (i >= arm_length)
					{
						movement m1;
						double x = (double)rand() / RAND_MAX;
						x = -0.5f + x * (1.0f);
						double y = (double)rand() / RAND_MAX;
						y = -0.5f + y * (1.0f);

						m1.velocity = Eigen::Vector3f(x, y, 0) / 40;
						m1.elasticity = 0.82f;
						movement_data.push_back(m1);
						data_list[i].getTrans().pretranslate(Eigen::Vector3f(0, 0, 0.9f));
					}
				}
				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/grass.obj");
				data().uniform_colors_index(4);
				data().shininess = 5.0f;
				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/Wall.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(0, 57, 0));
				data().uniform_colors_index(5);
				data().shininess = 100.0f;
				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/Pyramid.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(0, 52.0f, 0));

				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/ArrowRight.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(-15, 35.0f, 0));
				data_list[data_list.size() - 1].set_visible(false);
				right_arrow = data_list[data_list.size() - 1].id;

				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/ArrowLeft.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(15, 35.0f, 0));
				data_list[data_list.size() - 1].set_visible(false);
				left_arrow = data_list[data_list.size() - 1].id;

				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/Wall.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(0, -57, 0));
				data().uniform_colors_index(5);
				data().shininess = 100.0f;
				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/WallSides.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(57, 0, 0));
				data().uniform_colors_index(5);
				data().shininess = 100.0f;
				load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/WallSides.obj");
				data_list[data_list.size() - 1].getTrans().pretranslate(Eigen::Vector3f(-57, 0, 0));
				data().uniform_colors_index(5);
				data().shininess = 100.0f;
				//load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/cTower.obj");

				build_kd_trees();
				selected_data_index = saved_index;
				return 0;
			}

			int Viewer::load_meshs_ik()
			{
				Eigen::MatrixXd& V = data().V;
				Eigen::MatrixXi& F = data().F;
				// Find the bounding box
				Eigen::Vector3d m = V.colwise().minCoeff();
				Eigen::Vector3d M = V.colwise().maxCoeff();

				link_length = abs(M(1) - m(1));

				if (selected_data_index == 0)
				{
					arm_geo_center = Eigen::Vector3f((M(0) + m(0)) / 2, m(1), (M(2) + m(2)) / 2);
					parent_axis_coordinates[selected_data_index] = Eigen::Vector4f(0, 0.8, 0, 1);
					arm_root = Eigen::Vector4f(0, -0.8, 0, 1);
					arm_root_rotation = Eigen::Matrix4f::Identity();
				}
				else
				{
					data().MyTranslate(Eigen::Vector3f(0, link_length * (selected_data_index), 0), OBJECT_AXIS);
					parent_axis_coordinates[selected_data_index] = Eigen::Vector4f(0, 0.8 + link_length * (selected_data_index), 0, 1);
				}
				parent_axis_rotation[selected_data_index] = Eigen::Matrix4f::Identity();

				// Plot the mesh
				data().set_mesh(V, F);
				data().set_face_based(false);

				return 0;
			}

			int Viewer::load_meshs(int x)
			{
				std::ifstream in("./configuration.txt");
				int cnt = 0;
				if (!in)
				{
					std::cout << "can't open file configuration.txt!" << std::endl;
					return 0;
				}

				char str[255];
				int line_index = 0;
				while (in)
				{
					in.getline(str, 255);
					if (in)
					{
						int n = 0;
						if (line_index == 0)
						{
							n = arm_length;
						}
						else
						{
							n = x;
						}
						for (int p = 0; p < n; p++)
						{
							/*if (line_index == 0 && p == arm_length - 1)
							{
								load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/trex.obj");
							}
							else */if (!this->load_mesh_from_file(str))
							{
								//std::cout << ("Error loading mesh at %s .", str) << std::endl;
								return 0;
							}
							if(line_index == 1)
								data().Move(Eigen::Vector4f(2 * pow(-1.5f, p), 3 * pow(-1.5f, p), 0, 1));
							cnt++;
						}
						line_index++;
					}
				}
				
				return cnt;
			}

			void Viewer::gravity_handler(double timeDelta)
			{
				if (loading)
					return;
				Eigen::Vector3f gravityThisFrame = (timeDelta * gravity)/ 10000.0f;

				for (unsigned int i = 0; i < movement_data.size(); ++i)
				{
					if (data_list[arm_length + i].getTrans().translation().z() > 0.50f)
						movement_data[i].velocity += gravityThisFrame;
					else if (fabs(movement_data[i].velocity.z()) < 0.001f)
						movement_data[i].velocity.z() = 0.0f;
						

					if (data_list[i + arm_length].getTrans().translation().z() <= 0.4f)
						handle_ground_collision(i);
				}
				
			}

			void Viewer::snake_gravity_handler(double timeDelta)
			{
				Eigen::Vector3f gravityThisFrame = (timeDelta * gravity) / 10000.0f;
				snake_movement.velocity += gravityThisFrame;
			}

			void Viewer::handle_ground_collision(int index)
			{
				data_list[index + arm_length].getTrans().translation().z() = 0.4f;
				movement_data[index].velocity.z() *= -movement_data[index].elasticity;
			}

			IGL_INLINE bool Viewer::init_ds()
			{
				int saved_index = selected_data_index;

				for (int i = 0; i < data_list.size(); i++)
				{
					selected_data_index = i;
					ds* tmp = new ds(this);
					data_structures.push_back(tmp);
				}
				selected_data_index = saved_index;
				return true;
			}

			/*IGL_INLINE bool Viewer::collapse_5_percent(int n)
			{
				Eigen::MatrixXd& V = data_structures[selected_data_index]->V;
				Eigen::MatrixXi& F = data_structures[selected_data_index]->F;
				Eigen::MatrixXi& E = data_structures[selected_data_index]->E;
				Eigen::VectorXi& EMAP = data_structures[selected_data_index]->EMAP;
				Eigen::MatrixXi& EF = data_structures[selected_data_index]->EF;
				Eigen::MatrixXi& EI = data_structures[selected_data_index]->EI;
				typedef std::set<std::pair<double, int> > PriorityQueue;
				PriorityQueue& Q = data_structures[selected_data_index]->Q;
				std::vector<PriorityQueue::iterator >& Qit = data_structures[selected_data_index]->Qit;
				Eigen::MatrixXd& C = data_structures[selected_data_index]->C;
				int& num_collapsed = data_structures[selected_data_index]->num_collapsed;
				int edges_to_remove = std::ceil(0.05 * Q.size());
				bool something_collapsed = false;
				if (n == 0)
				{
					for (int i = 0; i < edges_to_remove; i++)
					{
						if (!my_collapse::my_collapse_e(V,
							F, E, EMAP, EF, EI, Q, Qit, C, this))
						{
							break;
						}
						something_collapsed = true;
						num_collapsed++;
					}
				}
				if (something_collapsed)
				{
					data().clear();
					data().set_mesh(V, F);
					data().set_face_based(true);
				}
				std::cout << "removed: " << num_collapsed << std::endl;
				return something_collapsed;
			}*/

			IGL_INLINE bool Viewer::collapse_edges(int num)
			{
				Eigen::MatrixXd& V = data_structures[selected_data_index]->V;
				Eigen::MatrixXi& F = data_structures[selected_data_index]->F;
				Eigen::VectorXi& EMAP = data_structures[selected_data_index]->EMAP;
				Eigen::MatrixXi& E = data_structures[selected_data_index]->E, & EF = data_structures[selected_data_index]->EF, & EI = data_structures[selected_data_index]->EI;
				typedef std::set<std::pair<double, int> > PriorityQueue;
				PriorityQueue& Q = data_structures[selected_data_index]->Q;
				std::vector<PriorityQueue::iterator >& Qit = data_structures[selected_data_index]->Qit;
				Eigen::MatrixXd& C = data_structures[selected_data_index]->C;
				int& num_collapsed = data_structures[selected_data_index]->num_collapsed;
				bool something_collapsed = false;

				for (int j = 0;j < num;j++)
				{
					const std::function<void(
						const int,
						const Eigen::MatrixXd&,
						const Eigen::MatrixXi&,
						const Eigen::MatrixXi&,
						const Eigen::VectorXi&,
						const Eigen::MatrixXi&,
						const Eigen::MatrixXi&,
						double&,
						Eigen::RowVectorXd&)>& cost_and_placement = shortest_edge_and_midpoint;

					if (!collapse_edge(
						shortest_edge_and_midpoint, V,
						F, E, EMAP, EF, EI, Q, Qit, C))
					{
						break;
					}
					something_collapsed = true;

					num_collapsed++;
				}
				return something_collapsed;
			}

			IGL_INLINE bool Viewer::quadric_error_handler()
			{
				int saved_index = selected_data_index;
				for (int i = 0; i < data_list.size(); i++)
				{
					selected_data_index = i;
					Eigen::MatrixXd* V = &data_structures[selected_data_index]->V;
					Eigen::MatrixXi* F = &data_structures[selected_data_index]->F;
					Eigen::VectorXi* EMAP = &data_structures[selected_data_index]->EMAP;
					Eigen::MatrixXi* E = &data_structures[selected_data_index]->E, * EF = &data_structures[selected_data_index]->EF, * EI = &data_structures[selected_data_index]->EI;
					typedef std::set<std::pair<double, int> > PriorityQueue;
					PriorityQueue* Q = &data_structures[selected_data_index]->Q;
					std::vector<PriorityQueue::iterator >* Qit = &data_structures[selected_data_index]->Qit;
					Eigen::MatrixXd* C = &data_structures[selected_data_index]->C;
					int* num_collapsed = &data_structures[selected_data_index]->num_collapsed;
					bool something_collapsed = false;
					double cost;
					Eigen::RowVectorXd p;
					quadric_error(1, *V, *F, *E, *EMAP, *EF, *EI, cost, p);
				}
				selected_data_index = saved_index;
				return true;
			}

			IGL_INLINE void Viewer::quadric_error(
				const int es,
				const Eigen::MatrixXd& V,
				const Eigen::MatrixXi& F,
				const Eigen::MatrixXi& E,
				const Eigen::VectorXi& EMAP,
				const Eigen::MatrixXi& EF,
				const Eigen::MatrixXi& EI,
				double& cost,
				Eigen::RowVectorXd& p)
			{
				typedef std::set<std::pair<double, int> > PriorityQueue;
				PriorityQueue& Q = data_structures[selected_data_index]->Q;
				std::vector<Eigen::Matrix4d>& Qv = data_structures[selected_data_index]->Qv;
				std::vector<PriorityQueue::iterator >& Qit = data_structures[selected_data_index]->Qit;
				Eigen::MatrixXd* C = &data_structures[selected_data_index]->C;
				std::vector<bool> check(V.rows(), false);
				for (int e = 0; e < E.rows(); e++)
				{
					std::vector<int> n_e0;
					std::vector<int> n_e1;
					if (!check.at(E(e, 0)))
					{
						n_e0 = igl::circulation(e, false, EMAP, EF, EI);
					}
					if (!check.at(E(e, 1)))
					{
						n_e1 = igl::circulation(e, true, EMAP, EF, EI);
					}
					/*
					for (int f : n_e0_e1)
					{
						if (E(EMAP(f), 0) == E(e, 0) || E(EMAP(f), 1) == E(e, 0) ||
							E(EMAP(f + F.rows()), 0) == E(e, 0) || E(EMAP(f + F.rows()), 1) == E(e, 0) ||
							E(EMAP(f + 2 * F.rows()), 0) == E(e, 0) || E(EMAP(f + 2 * F.rows()), 1) == E(e, 0))
						{
							n_e0.push_back(f);
							std::cout << "0 " << f << std::endl;
						}
						if (E(EMAP(f), 0) == E(e, 1) || E(EMAP(f), 1) == E(e, 1) ||
							E(EMAP(f + F.rows()), 0) == E(e, 1) || E(EMAP(f + F.rows()), 1) == E(e, 1) ||
							E(EMAP(f + 2 * F.rows()), 0) == E(e, 1) || E(EMAP(f + 2 * F.rows()), 1) == E(e, 1))
						{
							n_e1.push_back(f);
							std::cout << "1 " << f << std::endl;
						}
					}*/

					Eigen::Matrix4d qv0 = Eigen::Matrix4d::Zero();
					Eigen::Matrix4d qv1 = Eigen::Matrix4d::Zero();

					Eigen::Vector3d current_vertex_pos = V.row(E(e, 0));
					double x = current_vertex_pos.x(), y = current_vertex_pos.y(), z = current_vertex_pos.z();
					if (!check.at(E(e, 0)))
					{
						for (int f : n_e0)
						{
							Eigen::Vector3d face_normal = data().F_normals.row(f).normalized();
							double a = face_normal.x(), b = face_normal.y(), c = face_normal.z();
							double d = -(a * x + b * y + c * z);
							Eigen::Vector4d plane = Eigen::Vector4d(a, b, c, d);
							Qv.at(E(e, 0)) += plane * plane.transpose();
						}
						check.at(E(e, 0)) = true;
						//Qv.at(E(e, 0)) = qv0;
					}
					qv0 = Qv.at(E(e, 0));

					current_vertex_pos = V.row(E(e, 1));
					x = current_vertex_pos.x(), y = current_vertex_pos.y(), z = current_vertex_pos.z();
					if (!check.at(E(e, 1)))
					{
						for (int f : n_e1)
						{
							Eigen::Vector3d face_normal = data().F_normals.row(f).normalized();
							double a = face_normal.x(), b = face_normal.y(), c = face_normal.z();
							double d = -(a * x + b * y + c * z);
							Eigen::Vector4d plane = Eigen::Vector4d(a, b, c, d);
							Qv.at(E(e, 1)) += plane * plane.transpose();
						}
						//Qv.at(E(e, 1)) = qv1;
						check.at(E(e, 1)) = true;
					}
					qv1 = Qv.at(E(e, 1));


					Eigen::Matrix4d q_t = qv0 + qv1;
					Eigen::Matrix4d q_t_inv;

					//q_t.row(3) = Eigen::Vector4d(0, 0, 0, 1);

					double tmp[] = { q_t(0, 0), q_t(0, 1), q_t(0, 2), q_t(0, 3),
									 q_t(0, 1), q_t(1, 1), q_t(1, 2), q_t(1, 3),
									 q_t(0, 2), q_t(1, 2), q_t(2, 2), q_t(2, 3),
									 0, 0, 0, 1 };

					Eigen::Matrix4d q_t2 = Eigen::Map<Eigen::Matrix4d>(tmp).transpose();

					bool inv;
					q_t2.computeInverseWithCheck(q_t_inv, inv);
					if (inv)
					{
						p = q_t_inv * Eigen::Vector4d(0, 0, 0, 1);
					}
					else
					{
						p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
					}
					p = Eigen::Vector3d(p.x(), p.y(), p.z());

					Eigen::Vector4d v_t = Eigen::Vector4d(p.x(), p.y(), p.z(), 1);
					cost = ((v_t.transpose() * q_t) * v_t);

					C->row(e) = p;
					Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;
					/*if (selected_data_index == 1)
					{
						for (int f : n_e0)
							std::cout << f << " ";
						std::cout << std::endl;
						for (int f : n_e1)
							std::cout << f << " ";
						std::cout << std::endl;
						std::cout << cost << std::endl;
					}*/
				}
			}
			IGL_INLINE void Viewer::quadric_error_edge(
				const int e,
				const Eigen::MatrixXd& V,
				const Eigen::MatrixXi& F,
				const Eigen::MatrixXi& E,
				const Eigen::VectorXi& EMAP,
				const Eigen::MatrixXi& EF,
				const Eigen::MatrixXi& EI,
				double& cost,
				Eigen::RowVectorXd& p)
			{
				typedef std::set<std::pair<double, int> > PriorityQueue;
				PriorityQueue& Q = data_structures[selected_data_index]->Q;
				std::vector<Eigen::Matrix4d>& Qv = data_structures[selected_data_index]->Qv;
				std::vector<PriorityQueue::iterator >& Qit = data_structures[selected_data_index]->Qit;
				Eigen::MatrixXd* C = &data_structures[selected_data_index]->C;
				{
					std::vector<int> n_e0 = igl::circulation(e, false, EMAP, EF, EI);
					std::vector<int> n_e1 = igl::circulation(e, true, EMAP, EF, EI);
					/*
					for (int f : n_e0_e1)
					{
						if (E(EMAP(f), 0) == E(e, 0) || E(EMAP(f), 1) == E(e, 0) ||
							E(EMAP(f + F.rows()), 0) == E(e, 0) || E(EMAP(f + F.rows()), 1) == E(e, 0) ||
							E(EMAP(f + 2 * F.rows()), 0) == E(e, 0) || E(EMAP(f + 2 * F.rows()), 1) == E(e, 0))
						{
							n_e0.push_back(f);
							std::cout << "0 " << f << std::endl;
						}
						if (E(EMAP(f), 0) == E(e, 1) || E(EMAP(f), 1) == E(e, 1) ||
							E(EMAP(f + F.rows()), 0) == E(e, 1) || E(EMAP(f + F.rows()), 1) == E(e, 1) ||
							E(EMAP(f + 2 * F.rows()), 0) == E(e, 1) || E(EMAP(f + 2 * F.rows()), 1) == E(e, 1))
						{
							n_e1.push_back(f);
							std::cout << "1 " << f << std::endl;
						}
					}*/
					Eigen::Matrix4d qv0 = Eigen::Matrix4d::Zero();
					Eigen::Matrix4d qv1 = Eigen::Matrix4d::Zero();

					Eigen::Vector3d current_vertex_pos = V.row(E(e, 0));
					double x = current_vertex_pos.x(), y = current_vertex_pos.y(), z = current_vertex_pos.z();
					for (int f : n_e0)
					{
						Eigen::Vector3d face_normal = data().F_normals.row(f).normalized();
						double a = face_normal.x(), b = face_normal.y(), c = face_normal.z();
						double d = -(a * x + b * y + c * z);
						Eigen::Vector4d plane = Eigen::Vector4d(a, b, c, d);
						qv0 += plane * plane.transpose();
					}

					current_vertex_pos = V.row(E(e, 1));
					x = current_vertex_pos.x(), y = current_vertex_pos.y(), z = current_vertex_pos.z();
					for (int f : n_e1)
					{
						Eigen::Vector3d face_normal = data().F_normals.row(f).normalized();
						double a = face_normal.x(), b = face_normal.y(), c = face_normal.z();
						double d = -(a * x + b * y + c * z);
						Eigen::Vector4d plane = Eigen::Vector4d(a, b, c, d);
						qv1 += plane * plane.transpose();
					}
					Eigen::Matrix4d q_t = qv0 + qv1;
					Eigen::Matrix4d q_t_inv;

					double tmp[] = { q_t(0, 0), q_t(0, 1), q_t(0, 2), q_t(0, 3),
									 q_t(0, 1), q_t(1, 1), q_t(1, 2), q_t(1, 3),
									 q_t(0, 2), q_t(1, 2), q_t(2, 2), q_t(2, 3),
									 0, 0, 0, 1 };

					Eigen::Matrix4d q_t2 = Eigen::Map<Eigen::Matrix4d>(tmp).transpose();

					bool inv;
					q_t2.computeInverseWithCheck(q_t_inv, inv);
					if (inv) // DID NOT WORK FOR ME!
					{
						p = q_t_inv * Eigen::Vector4d(0, 0, 0, 1);
					}
					else
					{
						p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
					}
					p = Eigen::Vector3d(p.x(), p.y(), p.z());

					Eigen::Vector4d v_t = Eigen::Vector4d(p.x(), p.y(), p.z(), 1);
					cost = ((v_t.transpose() * q_t) * v_t);
				}
			}
		}
	} // end namespace
} // end namespace