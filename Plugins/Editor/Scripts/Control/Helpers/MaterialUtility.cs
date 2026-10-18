using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using InternalRealtimeCSG;
using UnityEngine.Rendering;


namespace RealtimeCSG
{
    public static class MaterialUtility
	{
		internal const string ShaderNameRoot						= "Hidden/CSG/internal/";
		internal const string SpecialSurfaceShaderName				= "specialSurface";
		internal const string SpecialSurfaceShaderID				= ShaderNameRoot + SpecialSurfaceShaderName;
		internal const string TransparentSpecialSurfaceShaderName	= "transparentSpecialSurface";
		internal const string TransparentSpecialSurfaceShaderID		= ShaderNameRoot + TransparentSpecialSurfaceShaderName;

		internal const string HiddenName				= "hidden";
		internal const string CulledName				= "culled";
		internal const string ColliderName				= "collider";
		internal const string TriggerName				= "trigger";
		internal const string ShadowOnlyName            = "shadowOnly";
		internal const string CastShadowsName			= "castShadows";
		internal const string ReceiveShadowsName        = "receiveShadows";

		internal const string HiddenMaterialName			= TransparentSpecialSurfaceShaderName + "_" + HiddenName;
		internal const string CulledMaterialName			= TransparentSpecialSurfaceShaderName + "_" + CulledName;
		internal const string ColliderMaterialName			= SpecialSurfaceShaderName + "_" + ColliderName;
		internal const string TriggerMaterialName			= TransparentSpecialSurfaceShaderName + "_" + TriggerName;
		internal const string ShadowOnlyMaterialName		= SpecialSurfaceShaderName + "_" + ShadowOnlyName;
		internal const string CastShadowsMaterialName		= SpecialSurfaceShaderName + "_" + CastShadowsName;
		internal const string ReceiveShadowsMaterialName	= SpecialSurfaceShaderName + "_" + ReceiveShadowsName;
		
		
		const string WallMaterialName	= "Wall";
		const string FloorMaterialName	= "Floor";
		const string WindowMaterialName	= "Window";
		const string MetalMaterialName	= "Metal";
		
		internal static Shader SpecialSurfaceShader;
		internal static Shader TransparentSpecialSurfaceShader;

		
		private static readonly Dictionary<string, Material> EditorMaterials = new Dictionary<string, Material>();

		private static bool _shadersInitialized;		//= false;
		private static int	_pixelsPerPointId			= -1;
		private static int	_lineThicknessMultiplierId	= -1; 
		private static int	_lineDashMultiplierId		= -1; 
		private static int	_lineAlphaMultiplierId		= -1;

		private static void ShaderInit()
		{
			_shadersInitialized = true;
	
			_pixelsPerPointId			= Shader.PropertyToID("_pixelsPerPoint");
			_lineThicknessMultiplierId	= Shader.PropertyToID("_thicknessMultiplier");
			_lineDashMultiplierId		= Shader.PropertyToID("_dashMultiplier");
			_lineAlphaMultiplierId		= Shader.PropertyToID("_alphaMultiplier");
			
			SpecialSurfaceShader			= Shader.Find(SpecialSurfaceShaderID);;
			TransparentSpecialSurfaceShader	= Shader.Find(TransparentSpecialSurfaceShaderID);
		}

		internal static Material GenerateEditorMaterial(string shaderName, string textureName = null, string materialName = null)
		{
			Material material;
			var name = shaderName + ":" + textureName;
			if (EditorMaterials.TryGetValue(name, out material))
			{
				// just in case one of many unity bugs destroyed the material
				if (!material)
				{
					EditorMaterials.Remove(name);
				} else
					return material;
			}

			if (materialName == null)
				materialName = name.Replace(':', '_');


			var shader = Shader.Find(ShaderNameRoot + shaderName);
			if (!shader)
			{
				Debug.LogWarning("Could not find internal shader: " + ShaderNameRoot + shaderName);
				return null;
			}

			material = new Material(shader)
			{
				name = materialName,
				hideFlags = HideFlags.DontSaveInBuild | HideFlags.DontUnloadUnusedAsset
			};
			if (textureName != null)
			{
				//string filename = "Assets/Plugins/RealtimeCSG/Editor/Resources/Textures/" + textureName + ".png";
                material.mainTexture = Resources.Load<Texture2D>( string.Format( "RealtimeCSG/Textures/{0}", textureName ) ); //AssetDatabase.LoadAssetAtPath<Texture2D>(filename);
				if (!material.mainTexture)
					Debug.LogWarning("Could not find internal texture: " + textureName);
			}
			EditorMaterials.Add(name, material);
			return material;
		}

		internal static Material GenerateEditorColorMaterial(Color color)
		{
			var name = "Color:" + color;

			Material material;
			if (EditorMaterials.TryGetValue(name, out material))
			{
				// just in case one of many unity bugs destroyed the material
				if (!material)
				{
					EditorMaterials.Remove(name);
				} else
					return material;
			}

			var shader = Shader.Find("Unlit/Color");
			if (!shader)
				return null;

			material = new Material(shader)
			{
				name		= name.Replace(':', '_'),
				hideFlags	= HideFlags.None | HideFlags.DontUnloadUnusedAsset
			};
			material.SetColor("_Color", color);

			EditorMaterials.Add(name, material);
			return material;
		}


		internal static bool EqualInternalMaterial(Material o, Material n)
		{
			return	((bool)o == (bool)n) &&
					o.shader == n.shader && 
					o.mainTexture == n.mainTexture && 
					//o.Equals(n);
					o.name == n.name;
		}

        private static bool TryGetShaderMainTex( Material material, out string mainTexTag )
        {
            if( material.HasProperty( "_Diffuse" ) )
            {
                mainTexTag = "_Diffuse";

                return true;
            }

            if( material.HasProperty( "_Albedo" ) )
            {
                mainTexTag = "_Albedo";

                return true;
            }

            if( material.HasProperty( "_BaseColorMap" ) )
            {
                mainTexTag = "_BaseColorMap";

                return true;
            }

            if( material.HasProperty( "_BaseMap" ) )
            {
                mainTexTag = "_BaseMap";

                return true;
            }

            if( material.HasProperty( "_MainTex" ) )
            {
                mainTexTag = "_MainTex";

                return true;
            }

            mainTexTag = "";

            return false;
        }

        // Only bundled defaults are passed here. User materials are never migrated or saved.
        private static void GenerateBuiltinPipelineMaterial( string name )
        {
            if (name != WallMaterialName && name != FloorMaterialName && name != WindowMaterialName && name != MetalMaterialName)
                return;

            Material material = Resources.Load<Material>( string.Format( "{0}{1}", "RealtimeCSG/Materials/", name ) );

            if( material == null )
            {
                Debug.LogWarningFormat( "Could not find material '{0}{1}.mat'. Not generating material '{0}{1}.mat' for the current render pipeline.", "RealtimeCSG/Materials/", name );

                return;
            }

            var pipelineAsset = GraphicsSettings.renderPipelineAsset;
            var targetShader = pipelineAsset ? pipelineAsset.defaultMaterial ? pipelineAsset.defaultMaterial.shader : null : Shader.Find("Standard");
            if (!targetShader)
            {
                Debug.LogWarning("Could not find the default material shader for the active render pipeline.");
                return;
            }

            // Cache values before changing the shader; broken bundled shader references may report no properties.
            var color = material.HasProperty("_BaseColor") ? material.GetColor("_BaseColor") :
                        material.HasProperty("_Color") ? material.GetColor("_Color") : Color.white;
            var targetTexture = Resources.Load<Texture2D>(string.Format("RealtimeCSG/Textures/{0}", name));
            var shaderChanged = material.shader != targetShader;
            var textureNeedsUpdate = targetTexture && (!TryGetShaderMainTex(material, out var currentMainTex) || material.GetTexture(currentMainTex) != targetTexture);
            if (!shaderChanged && !textureNeedsUpdate)
                return;

            material.shader = targetShader;
            if (shaderChanged)
            {
                if (material.HasProperty("_Color")) material.SetColor("_Color", color);
                if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
            }

            if (!TryGetShaderMainTex(material, out var mainTexTag))
            {
                Debug.LogWarningFormat("Default shader '{0}' for bundled material '{1}' has no supported base texture property.", targetShader.name, name);
                return;
            }

            if (targetTexture)
                material.SetTexture(mainTexTag, targetTexture);

            // Keep the bundled Metal material's intended metallic response when migrating shaders.
            if (name == MetalMaterialName && shaderChanged)
            {
                if (material.HasProperty("_Metallic"))
                {
                    material.SetFloat("_Metallic", 1f);
                    if (material.HasProperty("_Smoothness")) material.SetFloat("_Smoothness", 1f);
                    if (material.HasProperty("_MetallicGlossMap"))
                        material.SetTexture("_MetallicGlossMap", Resources.Load<Texture2D>("RealtimeCSG/Textures/checker_m"));
                }
                if (material.HasProperty("_DetailAlbedoMap"))
                    material.SetTexture("_DetailAlbedoMap", Resources.Load<Texture2D>("RealtimeCSG/Textures/checker"));
            }

            EditorUtility.SetDirty(material);
            // Save only when the bundled asset actually needed migration; getters then remain cheap and idempotent.
#if UNITY_2020_3_OR_NEWER && ! (UNITY_2020_3_0 || UNITY_2020_3_1 || UNITY_2020_3_2 || UNITY_2020_3_3 || UNITY_2020_3_4 || UNITY_2020_3_5 || UNITY_2020_3_6 || UNITY_2020_3_7 || UNITY_2020_3_8 || UNITY_2020_3_9 || UNITY_2020_3_10 || UNITY_2020_3_11 || UNITY_2020_3_12 || UNITY_2020_3_13 || UNITY_2020_3_14 || UNITY_2020_3_15)
            AssetDatabase.SaveAssetIfDirty(material);
#else
            AssetDatabase.SaveAssets();
#endif
        }

        internal static Material GetRuntimeMaterial( string materialName )
        {
            GenerateBuiltinPipelineMaterial(materialName);

            return Resources.Load<Material>( string.Format( "RealtimeCSG/Materials/{0}", materialName ) );
        }

        internal static PhysicMaterial GetRuntimePhysicMaterial(string materialName)
        {
            return Resources.Load<PhysicMaterial>( string.Format( "RealtimeCSG/Materials/{0}", materialName ) );
        }

        private static Material _defaultMaterial;
		public static Material DefaultMaterial
		{
			get
			{
				if (!_defaultMaterial)
				{
					var renderPipelineAsset = UnityEngine.Rendering.GraphicsSettings.renderPipelineAsset;
					if (renderPipelineAsset)
					{
#if UNITY_2019_1_OR_NEWER
						_defaultMaterial = renderPipelineAsset.defaultMaterial;
						if (!_defaultMaterial)
#elif UNITY_2017_2_OR_NEWER
						_defaultMaterial = renderPipelineAsset.GetDefaultMaterial();
						if (!_defaultMaterial)
#endif
							_defaultMaterial = GetColorMaterial(Color.magenta);
					} else
						_defaultMaterial = AssetDatabase.GetBuiltinExtraResource<Material>("Default-Diffuse.mat");
				}
				return _defaultMaterial;
			}
		}

        private static PhysicMaterial _defaultPhysicsMaterial;
        public static PhysicMaterial DefaultPhysicsMaterial
        {
            get
            {
                if (!_defaultPhysicsMaterial)
                {
                    _defaultPhysicsMaterial = GetRuntimePhysicMaterial("Default");
                    if (!_defaultPhysicsMaterial)
                        Debug.LogError("Default physics material is missing");
                }
                return _defaultPhysicsMaterial;
            }
        }
         
        private static readonly Dictionary<Color,Material> ColorMaterials = new Dictionary<Color, Material>();
		internal static Material GetColorMaterial(Color color)
		{
			Material material;
			if (ColorMaterials.TryGetValue(color, out material))
			{
				// just in case one of many unity bugs destroyed the material
				if (!material)
				{
					ColorMaterials.Remove(color);
				} else
					return material;
			}
			
			material = GenerateEditorColorMaterial(color);
			if (!material)
				return AssetDatabase.GetBuiltinExtraResource<Material>("Default-Diffuse.mat");

			ColorMaterials.Add(color, material);
			return material;
		}

		private static Material _missingMaterial;
		public static Material MissingMaterial
		{
			get
			{
				if (!_missingMaterial)
				{
					_missingMaterial = GetColorMaterial(Color.magenta);
					if (!_missingMaterial)
						return DefaultMaterial;
				}
				return _missingMaterial;
			}
		}

		private static Material _wallMaterial;
		public static Material WallMaterial
		{
			get
			{
				if (!_wallMaterial)
				{
					_wallMaterial = GetRuntimeMaterial(WallMaterialName);
					if (!_wallMaterial)
						return DefaultMaterial;
				}
				return _wallMaterial;
			}
		}

		private static Material _floorMaterial;
		public static Material FloorMaterial
		{
			get
			{
				if (!_floorMaterial)
				{
					_floorMaterial = GetRuntimeMaterial(FloorMaterialName);
					if (!_floorMaterial)
						return DefaultMaterial;
				}
				return _floorMaterial;
			}
		}

		private static Material _windowMaterial;
		public static Material WindowMaterial
		{
			get
			{
				if (!_windowMaterial)
				{
					_windowMaterial = GetRuntimeMaterial(WindowMaterialName);
					if (!_windowMaterial)
						return DefaultMaterial;
				}
				return _windowMaterial;
			}
		}

		private static Material _metalMaterial;
		public static Material MetalMaterial
		{
			get
			{
				if (!_metalMaterial)
				{
					_metalMaterial = GetRuntimeMaterial(MetalMaterialName);
					if (!_metalMaterial)
						return DefaultMaterial;
				}
				return _metalMaterial;
			}
		}

		private static float _lineThicknessMultiplier = 1.0f;
		internal static float LineThicknessMultiplier
		{
			get { return _lineThicknessMultiplier; }
			set
			{
				if (Math.Abs(_lineThicknessMultiplier - value) < MathConstants.EqualityEpsilon)
					return;
				_lineThicknessMultiplier = value;
			}
		}

		private static float _lineDashMultiplier = 1.0f;
		internal static float LineDashMultiplier
		{
			get { return _lineDashMultiplier; }
			set
			{
				if (Math.Abs(_lineDashMultiplier - value) < MathConstants.EqualityEpsilon)
					return;
				_lineDashMultiplier = value;
			}
		}

		private static float _lineAlphaMultiplier = 1.0f;
		internal static float LineAlphaMultiplier
		{
			get { return _lineAlphaMultiplier; }
			set
			{
				if (Math.Abs(_lineAlphaMultiplier - value) < MathConstants.EqualityEpsilon)
					return;
				_lineAlphaMultiplier = value;
			}
		}


		private static Material _zTestGenericLine;
		internal static Material ZTestGenericLine
		{
			get
			{
				if (!_zTestGenericLine)
					_zTestGenericLine = GenerateEditorMaterial("ZTestGenericLine");
				return _zTestGenericLine;
			}
		}

		internal static void InitGenericLineMaterial(Material genericLineMaterial)
		{
			if (!genericLineMaterial)
				return;
			
			if (!_shadersInitialized) ShaderInit();
			if (_pixelsPerPointId != -1)
			{
				genericLineMaterial.SetFloat(_pixelsPerPointId, EditorGUIUtility.pixelsPerPoint);
			}
			if (_lineThicknessMultiplierId != -1) genericLineMaterial.SetFloat(_lineThicknessMultiplierId, _lineThicknessMultiplier * EditorGUIUtility.pixelsPerPoint);
			if (_lineDashMultiplierId      != -1) genericLineMaterial.SetFloat(_lineDashMultiplierId,      _lineDashMultiplier);
			if (_lineAlphaMultiplierId	   != -1) genericLineMaterial.SetFloat(_lineAlphaMultiplierId,     _lineAlphaMultiplier);
		}

		private static Material _noZTestGenericLine;
		internal static Material NoZTestGenericLine
		{
			get
			{
				if (!_noZTestGenericLine)
				{
					_noZTestGenericLine = GenerateEditorMaterial("NoZTestGenericLine");
				}
				return _noZTestGenericLine;
			}
		}

		private static Material _coloredPolygonMaterial;
		internal static Material ColoredPolygonMaterial
		{
			get
			{
				if (!_coloredPolygonMaterial)
				{
					_coloredPolygonMaterial = GenerateEditorMaterial("customSurface");
				}
				return _coloredPolygonMaterial;
			}
		}

		private static Material _hiddenMaterial;
		internal static Material HiddenMaterial		{ get { if (!_hiddenMaterial) _hiddenMaterial = GenerateEditorMaterial(TransparentSpecialSurfaceShaderName, HiddenName, HiddenMaterialName); return _hiddenMaterial; } }

		private static Material _culledMaterial;
		internal static Material CulledMaterial		{ get { if (!_culledMaterial) _culledMaterial = GenerateEditorMaterial(TransparentSpecialSurfaceShaderName, CulledName, CulledMaterialName); return _culledMaterial; } }

		private static Material _colliderMaterial;
		internal static Material ColliderMaterial	{ get { if (!_colliderMaterial) _colliderMaterial = GenerateEditorMaterial(SpecialSurfaceShaderName, ColliderName, ColliderMaterialName); return _colliderMaterial; } }
		 
		private static Material _triggerMaterial;
		internal static Material TriggerMaterial	{ get { if (!_triggerMaterial) _triggerMaterial = GenerateEditorMaterial(TransparentSpecialSurfaceShaderName, TriggerName, TriggerMaterialName); return _triggerMaterial; } }

		private static Material _shadowOnlyMaterial;
		internal static Material ShadowOnlyMaterial { get { if (!_shadowOnlyMaterial) _shadowOnlyMaterial = GenerateEditorMaterial(SpecialSurfaceShaderName, ShadowOnlyName, ShadowOnlyMaterialName); return _shadowOnlyMaterial; } }

		private static Material _castShadowsMaterial;
		internal static Material CastShadowsMaterial { get { if (!_castShadowsMaterial) _castShadowsMaterial = GenerateEditorMaterial(SpecialSurfaceShaderName, CastShadowsName, CastShadowsMaterialName); return _castShadowsMaterial; } }

		private static Material _receiveShadowsMaterial;
		internal static Material ReceiveShadowsMaterial { get { if (!_receiveShadowsMaterial) _receiveShadowsMaterial = GenerateEditorMaterial(SpecialSurfaceShaderName, ReceiveShadowsName, ReceiveShadowsMaterialName); return _receiveShadowsMaterial; } }


		internal static Texture2D CreateSolidColorTexture(int width, int height, Color color)
		{
			var pixels = new Color[width * height];
			for (var i = 0; i < pixels.Length; i++)
				pixels[i] = color;
			var newTexture = new Texture2D(width, height);
            newTexture.hideFlags = HideFlags.DontUnloadUnusedAsset;
			newTexture.SetPixels(pixels);
			newTexture.Apply();
			return newTexture;
		}

		static Dictionary<Material, RenderSurfaceType> materialTypeLookup = new Dictionary<Material, RenderSurfaceType>();

		internal static void ResetMaterialTypeLookup()
		{
			materialTypeLookup.Clear();
		}

		internal static RenderSurfaceType GetMaterialSurfaceType(Material material)
		{
			if (object.ReferenceEquals(material, null))
				return RenderSurfaceType.Normal;

			RenderSurfaceType surfaceType;
			if (materialTypeLookup.TryGetValue(material, out surfaceType))
				return surfaceType;
			
			if (!material)
				return RenderSurfaceType.Normal;
			
			if (!_shadersInitialized) ShaderInit();

			var shader = material.shader;
			if (shader != SpecialSurfaceShader &&
				shader != TransparentSpecialSurfaceShader)
			{
				materialTypeLookup[material] = RenderSurfaceType.Normal;
				return RenderSurfaceType.Normal;
			}

			var shaderName = shader.name;
			if (shaderName != SpecialSurfaceShaderID &&
				shaderName != TransparentSpecialSurfaceShaderID)
			{
				materialTypeLookup[material] = RenderSurfaceType.Normal;
				return RenderSurfaceType.Normal;
			}

			switch (material.name)
			{
				case HiddenMaterialName:			{ surfaceType = RenderSurfaceType.Hidden;         materialTypeLookup[material] = surfaceType; return surfaceType; }
				case CulledMaterialName:			{ surfaceType = RenderSurfaceType.Culled;         materialTypeLookup[material] = surfaceType; return surfaceType; }
				case ColliderMaterialName:			{ surfaceType = RenderSurfaceType.Collider;       materialTypeLookup[material] = surfaceType; return surfaceType; }
				case TriggerMaterialName:			{ surfaceType = RenderSurfaceType.Trigger;        materialTypeLookup[material] = surfaceType; return surfaceType; }
				case ShadowOnlyMaterialName:		{ surfaceType = RenderSurfaceType.ShadowOnly;     materialTypeLookup[material] = surfaceType; return surfaceType; }
				case CastShadowsMaterialName:		{ surfaceType = RenderSurfaceType.CastShadows;    materialTypeLookup[material] = surfaceType; return surfaceType; }
				case ReceiveShadowsMaterialName:	{ surfaceType = RenderSurfaceType.ReceiveShadows; materialTypeLookup[material] = surfaceType; return surfaceType; }
			}
			
			materialTypeLookup[material] = RenderSurfaceType.Normal;
			return RenderSurfaceType.Normal;
		}

		internal static Material GetSurfaceMaterial(RenderSurfaceType renderSurfaceType)
		{
			switch (renderSurfaceType)
			{
				case RenderSurfaceType.Hidden:			return HiddenMaterial; 
				case RenderSurfaceType.Culled:			return CulledMaterial; 
				case RenderSurfaceType.Collider:		return ColliderMaterial; 
				case RenderSurfaceType.Trigger:			return TriggerMaterial; 
				case RenderSurfaceType.ShadowOnly:		return ShadowOnlyMaterial;
				case RenderSurfaceType.CastShadows:		return CastShadowsMaterial;
				case RenderSurfaceType.ReceiveShadows:	return ReceiveShadowsMaterial;
				case RenderSurfaceType.Normal:			return null;
				default:								return null;
			}
		}
	
	}
}
