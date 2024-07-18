/**
 * @file PID.hpp
 * @author
 * - Alex Au , RM2019 (cwauac@connect.ust.hk)
 * - John Lam, RM2020 (john.lck40@gmail.com)
 * @brief PID implementation with integral anti-windup.
 * @version 0.1
 * @date 2019-11-16
 *
 * @copyright Copyright (c) 2019
 *
 */
#pragma once
#ifdef USE_APP_CONFIG
#include "appConfig.h"
#endif

#ifndef TUNNING_PARAMETERS
#define TUNNING_PARAMETERS FALSE
#endif

namespace control
{
    /**
     * @brief PID Functor
     * @tparam float Return type, float normally.
     */
    template <typename T = float>
    class PID
    {
    public:
        /**
         * @brief PID Parameter data class.
         *
         */
        class Param
        {
        public:
            constexpr Param(float kp = 0,
                            float ki = 0,
                            float kd = 0,
                            float kb = 0,
                            float n = 0,
                            float integralDecay = 0,
                            float integralMax = 0)
                : kp(kp),
                  ki(ki),
                  kd(kd),
                  kb(kb),
                  n(n),
                  integralDecay(integralDecay),
                  integralMax(integralMax)
            {
            }
            /**
             * @brief Proportional factor
             */
            float kp;
            /**
             * @brief Integral factor
             */
            float ki;
            /**
             * @brief Derivative factor
             */
            float kd;
            /**
             * @brief Backward calculation factor.
             * @note Normally 1 if there is current feedback
             * (with same unit as the output).
             */
            float kb;
            /**
             * @brief Derivative low pass filter cut-off frequency.
             */
            float n;
            /**
             * @brief Integral decay term
             * @note Normally not needed.
             */
            float integralDecay;
            /**
             * @brief Integral max
             * @note Normally not needed if we have feedback
             */
            float integralMax;
        };
        /**
         * @brief Construct a new PID functor.
         * @param params PID parameters.
         */
        constexpr PID(const Param &params) : params(params) {}
        /**
         * @brief Apply the PID controller and return the output.
         * @param error Error
         * @param actualOutput Feedback input for back calculation
         * @return T PID controller output
         */
        T operator()(const T error, const T actualOutput)
        {
            // low pass filter applied on the derivative term
            return this->operator()(
                error, actualOutput, (error - sigma) * params.n);
        }
        /**
         * @brief Apply the PID controller and return the output.
         * @param error Error
         * @param actualOutput Feedback input for back calculation
         * @param derivative Derivative input, for example the speed for position
         * PID.
         * @return T PID controller output
         */
        T operator()(const T error, const T actualOutput, const T nothing)
        {
            count++;
            time_t t_now = time(NULL);
            double dt = (t_now - lastTick);
            lastTick = t_now;
            sigma += derivative * dt;
            outputIntegral +=
                (error * params.ki * dt);
            if (count == 100)
            {
                count = 0;
                outputIntegral = 0;
            }

            if (params.integralMax)
            {
                if (outputIntegral > params.integralMax)
                    outputIntegral = params.integralMax;
                else if (outputIntegral < -params.integralMax)
                    outputIntegral = -params.integralMax;
            }
            if (abs(params.kd) > 0 && dt > 0)
            {
                if (isnan(last_derivative))
                {
                    derivative = 0;
                    last_derivative = 0;
                }
                else
                {
                    derivative = (error - last_error) / dt;
                }
                last_error = error;
                last_derivative = derivative;
            }

#if TUNNING_PARAMETERS
            pOutput = error * params.kp;
            dOutput = derivative * params.kd;
#endif
            output = error * params.kp + outputIntegral + derivative * params.kd;
            return output;
        }
        /**
         * @brief Reset the memory of the PID controller. Should be called when the
         * motor first connect to the board, or after reconnection.
         */
        void reset()
        {
            outputIntegral = 0;
            last_derivative = float('nan');
            time_t lastTick = time(NULL);
        }

    public:
        const Param &params;
        time_t lastTick = time(NULL);
        int count = 0;
        T output = 0;
        T sigma = 0;
        /**
         * @brief Output from integral term.
         */

        T outputIntegral = 0;
        T derivative = 0;
        T last_derivative = float('nan');
        T last_error = 0;
#if TUNNING_PARAMETERS
        /**
         * @brief Output from proportional term.
         */
        T pOutput = 0;
        /**
         * @brief Output from derivative term.
         */
        T dOutput = 0;
#endif
    };

} // namespace Control