

#ifndef STELLA_VSLAM_OPTIMIZE_POSE_OPTIMIZER_EXTENDED_LINE_H
#define STELLA_VSLAM_OPTIMIZE_POSE_OPTIMIZER_EXTENDED_LINE_H

namespace stella_vslam
{

    namespace data
    {
        class frame;
    }

    namespace optimize
    {

        class pose_optimizer_extended_line
        {
        public:
            /**
             * Constructor
             * @param num_trials
             * @param num_each_iter
             */
            explicit pose_optimizer_extended_line(const unsigned int num_trials = 4, const unsigned int num_each_iter = 10);

            /**
             * Destructor
             */
            virtual ~pose_optimizer_extended_line() = default;

            /**
             * Perform pose optimization
             * @param frm
             * @return
             */
            unsigned int optimize(data::frame &frm) const;

        private:
            //! Number of robust optimization trials
            const unsigned int num_trials_ = 4;

            //! Number of optimization iterations
            const unsigned int num_each_iter_ = 10;
        };
    }
}

#endif
