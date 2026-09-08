using System;
using System.Text;

namespace PyLoN.Domain
{
    /// <summary>Pure rules for sensor IDs. This layer has no KSP or Unity dependency.</summary>
    internal static class SensorIdentity
    {
        public const int MaxLength = 64;

        public static string Normalize(string value)
        {
            if (string.IsNullOrWhiteSpace(value))
            {
                return string.Empty;
            }

            var builder = new StringBuilder(Math.Min(value.Length, MaxLength));
            var previousUnderscore = false;
            for (var index = 0; index < value.Length && builder.Length < MaxLength; index++)
            {
                var character = char.ToLowerInvariant(value[index]);
                var accepted = character >= 'a' && character <= 'z' ||
                    character >= '0' && character <= '9';
                if (accepted)
                {
                    builder.Append(character);
                    previousUnderscore = false;
                }
                else if (!previousUnderscore && builder.Length > 0)
                {
                    builder.Append('_');
                    previousUnderscore = true;
                }
            }

            var normalized = builder.ToString().Trim('_');
            if (!string.IsNullOrEmpty(normalized) && normalized[0] >= '0' && normalized[0] <= '9')
            {
                normalized = "_" + normalized;
            }
            return normalized.Length <= MaxLength
                ? normalized
                : normalized.Substring(0, MaxLength);
        }
    }
}
