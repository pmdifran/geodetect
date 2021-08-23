#pragma once
#include <vector>

namespace GeoDetection
{
	class ScalarField : public std::vector<float>
	{
	private:
		const char* m_name = "Default";

	public:
		using Ptr = std::shared_ptr<ScalarField>;

		ScalarField() = default;
		ScalarField(const char* name) : m_name(name) {}

		ScalarField(const ScalarField& scalar_field) = default;
		ScalarField(ScalarField&& scalar_field) = default;

		ScalarField& operator=(const ScalarField&) = default;
		ScalarField& operator=(ScalarField&&) = default;

		inline void setName(const char* name) { m_name = name; }
		inline const char* const getName() const { return m_name; }


		
	};
}