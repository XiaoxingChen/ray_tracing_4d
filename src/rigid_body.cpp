#include "rigid_body.h"
#include "rotation.h"
#include "axis_aligned_bounding_box.h"
#include "bounding_volume_hierarchy.h"
#include "primitive_mesh_tree.h"
#include "rigid_transform.h"

#include <memory>



namespace rtc
{
    class Sphere: public RigidBody
    {
        public:
            Sphere(
                const std::vector<FloatType>& center,
                FloatType radius=3)
                :center_(center), radius_(radius){}

            Sphere(size_t dimension=3)
                :center_({0,0,0}), radius_(2){}

            virtual HitRecordPtr hit(const Ray& ray) const
            {
                Vec oc = center_ - ray.origin();

                Vec closest_pt(ray.origin() + ray.direction() * oc.dot(ray.direction()));
                FloatType dist = (closest_pt - center_).norm(2);

                if(dist >= radius_) return nullptr;
                FloatType t = oc.dot(ray.direction());
                if(t < 0) return nullptr;
                if(oc.norm() > radius_ + eps())
                    t -= sqrt(radius_ * radius_ - dist * dist);
                else
                    t += sqrt(radius_ * radius_ - dist * dist);

                if (!ray.valid(t)) return nullptr;
                return std::make_shared<HitRecord>(t, ray(t), ray(t) - center_);
            }
            virtual std::string str() const
            {
                return std::string("c: ") + std::to_string(center_(0)) + ", " + std::to_string(center_(1)) + ", " + std::to_string(center_(2));
            }

            // virtual Vec center() const { return center_; }
            virtual AABB aabb() const
            {
                return AABB(center_ - radius_, center_ + radius_);
            }

        private:
            Vec center_;
            FloatType radius_;
    };

    class Rectangle: public RigidBody
    {
        public:
            Rectangle(
                const std::vector<FloatType>& center,
                const std::vector<FloatType>& radius,
                const Rotation& orientation)
                :center_(center), radius_(radius), orientation_(orientation){}

            Rectangle(size_t dim=3)
                :center_(Vec::zeros(dim)), radius_(Vec::ones(dim)*.5), orientation_(Rotation::Identity(dim)){}

            virtual HitRecordPtr hit(const Ray& ray) const
            {
                Ray moved_ray(
                    orientation_.inv().apply(ray.origin() - center_),
                    orientation_.inv().apply(ray.direction()));
                auto t_in_out = AxisAlignedBoundingBox::hit(moved_ray, -radius_, radius_);
                bool hit = (t_in_out[0] < t_in_out[1]) && (ray.valid(t_in_out[0]) || ray.valid(t_in_out[1]));
                if(!hit) return nullptr;

                auto p_record = std::make_shared<HitRecord>(ray.origin().size());
                #if 0
                if(ray.valid(t_in_out[0]))
                {
                    std::cout << "outside" << std::endl;
                }
                else
                {
                    std::cout << "inside, t_in:" << t_in_out[0] << std::endl;
                }
                #endif

                FloatType hit_t = ray.valid(t_in_out[0]) ? t_in_out[0] : t_in_out[1];

                // if(! ray.valid(hit_t)) return nullptr;

                FloatType min_dist = radius_(0);
                size_t min_axis = 0;
                Vec dist0(radius_ - moved_ray(hit_t));
                Vec dist1(-radius_ - moved_ray(hit_t));
                for(size_t a = 0; a < center_.size(); a++)
                {
                    if(std::min(fabs(dist0(a)), fabs(dist1(a))) > min_dist) continue;
                    min_axis = a;
                    min_dist = std::min(fabs(dist0(a)), fabs(dist1(a)));
                }
                Vec norm_vec(Vec::zeros(center_.size()));
                norm_vec(min_axis) = -moved_ray.direction()(min_axis);

                p_record->n = static_cast<Vec>(orientation_.apply(norm_vec));
                // std::cout << "n: " << p_record->n.T().str() << "dist: " << min_dist << std::endl;
                p_record->t = hit_t;
                p_record->p = ray(hit_t);
                // p_record->n = norm_vec;
                return p_record;
            }

            // virtual Vec center() const { return center_; }
            virtual AABB aabb() const
            {
                size_t dim(center_.size());
                Mat vertices(radius_.matmul(Vec::ones(1 << dim).T()));
                for(size_t i = 0; i < dim; i++)
                {
                    for(size_t j = 0; j < vertices.shape(1); j++)
                    {
                        if((j & (1 << i)) > 0)
                            vertices(i, j) -= (2 * radius_(i));
                    }
                }
                vertices = orientation_.apply(vertices);
                vertices += center_.matmul(Vec::ones(1 << dim).T());

                AABB box(dim);
                box.extend(vertices);
                return box;
            }

        private:
            Vec center_;
            Vec radius_;
            Rotation orientation_;
    };

class PrimitiveMesh: public RigidBody
{
public:
    PrimitiveMesh(
        const Vec& position,
        const Rotation& orientation,
        std::shared_ptr<Mat>& vertices,
        std::shared_ptr<Matrix<size_t>>& indices)
        :pose_(position, orientation),
        tree_(vertices, indices), aabb_(position.size())
    {
        auto global_frame_vertices = pose_.apply(*vertices);
        aabb_.extend(global_frame_vertices);
        tree_.build(1, /* verbose */ false);
    }

    virtual HitRecordPtr hit(const Ray& ray) const
    {
        Ray local_ray(pose_.inv().apply(ray.origin()), pose_.rotation().inv().apply(ray.direction()));
        auto results = tree_.hit(local_ray, bvh2::eCloseHit);
        if(results.empty())
            return nullptr;
        auto ret = std::make_shared<HitRecord>(results.front());
        ret->p = pose_.apply(results.front().prim_coord_hit_p);
        ret->n = pose_.rotation().apply(results.front().n);
        return ret;
    }

    // virtual Vec center() const { return position_; }
    virtual AABB aabb() const
    {
        return aabb_;
    }

    size_t dim() const { return pose_.dim(); }
private:

    RigidTrans pose_;
    bvh2::PrimitiveMeshTree tree_;
    AxisAlignedBoundingBox aabb_;
};

class PolygonPrimitive: public RigidBody
{
public:
    PolygonPrimitive(std::shared_ptr<Mat>& vertex_buffer, std::shared_ptr<Matrix<size_t>>& indices, size_t prim_idx)
        :p_vertex_buffer_(vertex_buffer), p_vertex_index_buffer_(indices), prim_idx_(prim_idx)
        // , norm_(indices.size())
        {   }

    std::vector<size_t> indices() const
    {
        return (*p_vertex_index_buffer_)(Col(prim_idx_)).asVector();
    }

    virtual HitRecordPtr hit(const Ray& ray) const
    {
        auto p_record = hitPrimitivePolygon(ray, p_vertex_buffer_, indices());
        if(p_record)
        {
            p_record->prim_idx = prim_idx_;
            p_record->prim_coord_hit_p = p_record->p;
        }
        return p_record;
    }

    virtual void multiHit(const Ray& ray, std::vector<HitRecordPtr>& records) const
    {
        records.push_back(hitPrimitivePolygon(ray, p_vertex_buffer_, indices()));
    }

    size_t dim() const { return indices().size(); }

    virtual AABB aabb() const
    {
        AABB box(dim());
        for(auto & idx: indices())
        {
            box.extend((*p_vertex_buffer_)(Col(idx)));
        }
        return box;
    }
private:
    std::shared_ptr<Mat> p_vertex_buffer_;
    std::shared_ptr<Matrix<size_t>> p_vertex_index_buffer_;
    size_t prim_idx_;
};


//
// A N-Dimensional rigid body, generated by sweep an N-1 Dimensional
// rigid body along N-th axis with length h.
//
// e.g. A cyllinder can be generated by sweep a circle along Z-axis.
class Prism: public RigidBody
{
public:
    Prism(const Vec& p, const Rotation& r, FloatType h,
        std::shared_ptr<Mat>& vertex_buffer,
        std::shared_ptr<Matrix<size_t>>& vertex_index_buffer);

    size_t dim() const { return position_.size(); }

    virtual RigidBody::HitRecordPtr hit(const Ray& ray) const;

    virtual AABB aabb() const { return aabb_; }

private:
    Vec position_;
    Rotation orientation_;
    FloatType half_h_;

    bvh2::PrimitiveMeshTree tree_;
    AABB aabb_;
};

Prism::Prism(const Vec& p, const Rotation& r, FloatType h,
        std::shared_ptr<Mat>& vertex_buffer,
        std::shared_ptr<Matrix<size_t>>& vertex_index_buffer):
        position_(p), orientation_(r),
        half_h_(0.5 * h),
        tree_(vertex_buffer, vertex_index_buffer),
        aabb_(p.size())
{
    tree_.build(1, true);

    Mat vertices_full_dim({dim(), vertex_buffer->shape(1) * 2});
    vertices_full_dim(Block({end() - 1, end()}, {0, vertex_buffer->shape(1)})) += half_h_;
    vertices_full_dim(Block({end() - 1, end()}, {vertex_buffer->shape(1), end()})) -= half_h_;
    vertices_full_dim.setBlock(0, 0, *vertex_buffer);
    vertices_full_dim.setBlock(0, vertex_buffer->shape(1), *vertex_buffer);

    auto trans_v = rigidBodyTransform(position_, orientation_, vertices_full_dim);
    aabb_.extend(trans_v);
}

RigidBody::HitRecordPtr Prism::hit(const Ray& ray) const
{
    size_t h_axis = dim() - 1;

    //
    // create sub_ray
    Ray local_ray(orientation_.inv().apply(ray.origin() - position_), orientation_.inv().apply(ray.direction()));
    Vec sub_origin(dim() - 1);
    Vec sub_dir(dim() - 1);
    for(size_t i = 0; i < dim() - 1; i++)
    {
        sub_origin(i) = local_ray.origin()(i);
        sub_dir(i) = local_ray.direction()(i);
    }
    Ray sub_ray(sub_origin, sub_dir, -tMax(), tMax());

    //
    // eliminate longitudinal miss
    if(abs(local_ray.direction()(h_axis)) < eps() && abs(local_ray.origin()(h_axis)) > half_h_)
        return nullptr;

    FloatType h_hit_t_min;
    FloatType h_hit_t_max;

    {
        auto t0 = (half_h_ - local_ray.origin()(h_axis)) / local_ray.direction()(h_axis);
        auto t1 = (-half_h_ - local_ray.origin()(h_axis)) / local_ray.direction()(h_axis);
        h_hit_t_min = std::min(t0, t1);
        h_hit_t_max = std::max(t0, t1);
        // if(!local_ray.valid(h_hit_t_min) && !local_ray.valid(h_hit_t_max))
        //     return nullptr;
    }

    //
    // check subspace hit
    auto records = tree_.hit(sub_ray, bvh2::eMultiHit);

    if(records.empty()) return nullptr;

    std::sort(records.begin(), records.end(), [](auto a, auto b){ return a.t < b.t; });
    for(auto& rec : records) rec.t /= sub_dir.norm();
    for(auto it = records.begin() + 1; it != records.end();)
    {
        it = abs(it->t - (it - 1)->t) < 10*eps() ? records.erase(it) : it + 1;
    }

    if(records.front().t > h_hit_t_max || records.back().t < h_hit_t_min)
        return nullptr;

    auto result = std::make_shared<RigidBody::HitRecord>(dim());
    for(size_t i = 0; i + 1< records.size(); i+= 2)
    {
        // records.at(i) is "in edge"
        //  && (local_ray.valid(h_hit_t_min) || local_ray.valid(h_hit_t_max))
        if(h_hit_t_min < records.at(i).t && records.at(i).t < h_hit_t_max && local_ray.valid(records.at(i).t))
        {
            Vec local_norm = Vec::zeros(dim());
            local_norm.setBlock(0,0, records.at(i).n);
            result->n = orientation_.apply(local_norm);

            result->t = records.at(i).t;
            result->prim_coord_hit_p = local_ray(result->t) + 1e-3 * local_norm;
            result->prim_idx = records.at(i).prim_idx;
            result->p = orientation_.apply(result->prim_coord_hit_p) + position_;
            // std::cout << "lateral hit" << std::endl;
            return result;
        }

        if((records.at(i).t < h_hit_t_min) && (h_hit_t_min < records.at(i + 1).t) && local_ray.valid(h_hit_t_min))
        {
            Vec local_norm = Vec::zeros(dim());
            local_norm(h_axis) = (local_ray.direction()(h_axis) > 0) ? -1: 1;
            result->n = orientation_.apply(local_norm);

            result->t = h_hit_t_min;
            result->prim_coord_hit_p = local_ray(result->t) + 1e-3 * local_norm;
            result->prim_idx = tree_.vertexBuffer().shape(1);
            result->p = orientation_.apply(result->prim_coord_hit_p) + position_;
            // std::cout << "longitudinal hit" << std::endl;
            return result;
        }
    }

    return nullptr;
}

#if 1
    RigidBodyPtr RigidBody::choose(Types type, size_t dimension, const std::vector<FloatType>& args)
    {
        if(type == RigidBody::SPHERE)
            return std::make_shared<Sphere>(std::vector<FloatType>(args.begin(), args.begin() + dimension), args.at(dimension));

        if(type == RigidBody::RECTANGLE)
        {
            Mat plane({dimension,2}, std::vector<FloatType>(args.begin() + 2*dimension, args.begin() + 4*dimension));
            return std::make_shared<Rectangle>(
                std::vector<FloatType>(args.begin(), args.begin() + dimension), //center
                std::vector<FloatType>(args.begin() + dimension, args.begin() + 2*dimension), //radius
                Rotation::fromPlaneAngle(plane(Col(0)), plane(Col(1)), args.back())
                ); //rotation
        }


        return std::make_shared<Sphere>();
    }
    #endif

    RigidBodyPtr RigidBody::choose(Types type, VecIn position, const Rotation& orientation, const std::vector<FloatType>& args)
    {
        if(position.size() != orientation.dim())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        if(type == RigidBody::SPHERE)
            return std::make_shared<Sphere>(position, args.at(0));

        if(type == RigidBody::RECTANGLE)
        {
            return std::make_shared<Rectangle>(
                position, //center
                args, //radius
                orientation); //rotation
        }


        return std::make_shared<Sphere>();
    }

    RigidBodyPtr RigidBody::createPrimitiveMesh(
        const Vec& position,
        const Rotation& orientation,
        std::shared_ptr<Mat>& vertices,
        std::shared_ptr<Matrix<size_t>>& indices)
    {
        return std::make_shared<PrimitiveMesh>(position, orientation, vertices, indices);
    }

    RigidBodyPtr RigidBody::createPolygonPrimitive(
        std::shared_ptr<Mat> vertex_buffer,
        std::shared_ptr<Matrix<size_t>>& indices,
        size_t prim_idx)
    {
        return std::make_shared<PolygonPrimitive>(vertex_buffer, indices, prim_idx);
    }

    RigidBodyPtr RigidBody::createPrism(const Vec& p, const Rotation& r, FloatType h,
        std::shared_ptr<Mat>& vertex_buffer,
        std::shared_ptr<Matrix<size_t>>& vertex_index_buffer)
    {
        return std::make_shared<Prism>(p, r, h, vertex_buffer, vertex_index_buffer);
    }

} // namespace rtc
