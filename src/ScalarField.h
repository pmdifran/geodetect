#pragma once
#include <vector>
#include <memory>

namespace GeoDetection
{
	//Scalar field class (wrapper of std::vector<float> with name)
	class ScalarField
	{
	public:
		const char* name = "Default";
		std::vector<float> data;

		//Constructors
		ScalarField() = default;

		ScalarField(int64_t size) :
			data(std::vector<float>(size)) {}

		ScalarField(int64_t size, const char* name) : 
			data(std::vector<float>(size)),
			name(name) {}

		ScalarField(int64_t size, float init_value, const char* name) :
			data(std::vector<float>(size, init_value)),
			name(name) {}

		ScalarField(std::vector<float>& vec) :
			data(vec) {}

		ScalarField(std::vector<float>&& vec) :
			data(std::move(vec)) {}

		//Copy & Move Ctors (placeholders)
		ScalarField(const ScalarField& scalar_field) = default;
		ScalarField(ScalarField&& scalar_field) = default;

		//Assignment Operators (placeholders)
		ScalarField& operator=(const ScalarField&) = default;
		ScalarField& operator=(ScalarField&& rhs) = default;

		//Assignment Operators with std::vector
		ScalarField& operator=(std::vector<float>& vec)
		{
			data = vec;
			return *this;
		}

		ScalarField& operator=(std::vector<float>&& vec)
		{
			data = std::move(vec);
			return *this;
		}

		//Vector Access Operators
		float& operator[](const size_t index) { return data[index]; }

		const float& operator[](const size_t index) const { return data[index];}

		//Getters 
	public:
		const size_t size() const { return data.size(); }
		const std::vector<float>::const_iterator begin() const { return data.begin(); }
		const std::vector<float>::const_iterator end() const { return data.end(); }

		const std::vector<float>::const_reverse_iterator rbegin() const { return data.rbegin(); }
		const std::vector<float>::const_reverse_iterator rend() const { return data.rend(); }

		//Modifyers
	public:
		//Modify scalar field data with min-max normalization
		void normalizeMinMax();

		//Make all NaN data equal to the max
		void NaNtoMax();

		//Make all NaN data equal to the min
		void NaNtoMin();

		//Make all NaN data equal to zero
		void NaNtoZero();
	};
}