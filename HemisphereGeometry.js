/*global define*/
define([
  './BoundingSphere',
  './Cartesian2',
  './Cartesian3',
  './ComponentDatatype',
  './defaultValue',
  './defined',
  './DeveloperError',
  './Ellipsoid',
  './Geometry',
  './GeometryAttribute',
  './GeometryAttributes',
  './IndexDatatype',
  './Math',
  './PrimitiveType',
  './VertexFormat'
], function (BoundingSphere,
             Cartesian2,
             Cartesian3,
             ComponentDatatype,
             defaultValue,
             defined,
             DeveloperError,
             Ellipsoid,
             Geometry,
             GeometryAttribute,
             GeometryAttributes,
             IndexDatatype,
             CesiumMath,
             PrimitiveType,
             VertexFormat) {
  "use strict";


  var HemisphereGeometry = function (options) {

    options = defaultValue(options, defaultValue.EMPTY_OBJECT);

    var radius = defaultValue(options.radius, 10000);
    var minRange = defaultValue(options.minRange, 300);
    var stackPartitions = defaultValue(options.stackPartitions, 10);
    var slicePartitions = defaultValue(options.slicePartitions, 10);
    var angleAz = defaultValue(options.angleAz, 90);
    var minEl = defaultValue(options.minEl, 3);
    var maxEl = defaultValue(options.maxEl, 85);
    var vertexFormat = defaultValue(options.vertexFormat, VertexFormat.DEFAULT);

    //>>includeStart('debug', pragmas.debug);
    if (slicePartitions < 3) {
      throw new DeveloperError('options.slicePartitions cannot be less than three.');
    }
    if (stackPartitions < 3) {
      throw new DeveloperError('options.stackPartitions cannot be less than three.');
    }
    if (!defined(radius) || radius < 0) {
      throw new DeveloperError('options.radius cannot be less than zero.');
    }
    //>>includeEnd('debug');

    this._radius = radius;
    this._minRange = minRange;
    this._stackPartitions = stackPartitions;
    this._slicePartitions = slicePartitions;
    this._angleAz = angleAz;
    this._minEl = minEl;
    this._maxEl = maxEl;
    this._vertexFormat = VertexFormat.clone(vertexFormat);
    this._workerName = 'createHemisphereGeometry';
  };

  HemisphereGeometry.packedLength = VertexFormat.packedLength + 7;

  HemisphereGeometry.pack = function (value, array, startingIndex) {
    //>>includeStart('debug', pragmas.debug);
    if (!defined(value)) {
      throw new DeveloperError('value is required');
    }
    if (!defined(array)) {
      throw new DeveloperError('array is required');
    }
    //>>includeEnd('debug');

    startingIndex = defaultValue(startingIndex, 0);

    VertexFormat.pack(value._vertexFormat, array, startingIndex);
    startingIndex += VertexFormat.packedLength;

    array[startingIndex++] = value._radius;
    array[startingIndex++] = value._minRange;
    array[startingIndex++] = value._stackPartitions;
    array[startingIndex++] = value._slicePartitions;
    array[startingIndex++] = value._angleAz;
    array[startingIndex++] = value._minEl;
    array[startingIndex] = value._maxEl;
  };

  var scratchVertexFormat = new VertexFormat();
  var scratchOptions = {
    vertexFormat: scratchVertexFormat,
    radius: undefined,
    minRange: undefined,
    stackPartitions: undefined,
    slicePartitions: undefined,
    angleAz: undefined,
    minEl: undefined,
    maxEl: undefined
  };
  HemisphereGeometry.unpack = function (array, startingIndex, result) {
    //>>includeStart('debug', pragmas.debug);
    if (!defined(array)) {
      throw new DeveloperError('array is required');
    }
    //>>includeEnd('debug');

    startingIndex = defaultValue(startingIndex, 0);

    var vertexFormat = VertexFormat.unpack(array, startingIndex, scratchVertexFormat);
    startingIndex += VertexFormat.packedLength;

    var radius = array[startingIndex++];
    var minRange = array[startingIndex++];
    var stackPartitions = array[startingIndex++];
    var slicePartitions = array[startingIndex++];
    var angleAz = array[startingIndex++];
    var minEl = array[startingIndex++];
    var maxEl = array[startingIndex];

    if (!defined(result)) {
      scratchOptions.radius = radius;
      scratchOptions.minRange = minRange;
      scratchOptions.stackPartitions = stackPartitions;
      scratchOptions.slicePartitions = slicePartitions;
      scratchOptions.angleAz = angleAz;
      scratchOptions.minEl = minEl;
      scratchOptions.maxEl = maxEl;
      return new HemisphereGeometry(scratchOptions);
    }

    result._vertexFormat = VertexFormat.clone(vertexFormat, result._vertexFormat);
    result._radius = radius;
    result._minRange = minRange;
    result._stackPartitions = stackPartitions;
    result._slicePartitions = slicePartitions;
    result._angleAz = angleAz;
    result._minEl = minEl;
    result._maxEl = maxEl;

    return result;
  };
  HemisphereGeometry.createGeometry = function (hemisphereGeometry) {
    var radius = hemisphereGeometry._radius;
    var minRange = hemisphereGeometry._minRange;
    var angleAz = hemisphereGeometry._angleAz;
    var minEl = hemisphereGeometry._minEl;
    var maxEl = hemisphereGeometry._maxEl;
    var vertexFormat = hemisphereGeometry._vertexFormat;
    var slicePartitions = hemisphereGeometry._slicePartitions;
    var stackPartitions = hemisphereGeometry._stackPartitions;

    //Top and Bottom
    var TBvertexCount = ((slicePartitions + 1) * 4);
    var TBindex = 0;
    var TBindexN = 0;

    //Left and Right
    var LRvertexCount = ((stackPartitions + 1) * 4);
    var LRindex = TBvertexCount * 3;
    var LRindexN = TBvertexCount * 3;

    //Inner and Outer
    var IOvertexCount = ((stackPartitions + 1) * (slicePartitions + 1)) * 2;
    var IOindex = (TBvertexCount + LRvertexCount) * 3;
    var IOindexN = (TBvertexCount + LRvertexCount) * 3;

    //Final array
    var vertexCount = TBvertexCount + LRvertexCount + IOvertexCount;
    var positions = new Float64Array(vertexCount * 3);
    var normals = new Float32Array(vertexCount * 3);

    var angleEl = maxEl - minEl;
    var aDiv = (angleEl / stackPartitions);
    var stEl = (90 - maxEl) / aDiv;
    var stStop = stEl + stackPartitions;

    //vertices and normals

    for (var lat = stEl; lat <= stStop; lat += 1) {
      //Primary vertex calculation
      var theta = (Math.PI * lat) / (180 / aDiv);
      var sinTheta = Math.sin(theta);
      var cosTheta = Math.cos(theta);

      //Calculation for normals +/-
      var thetaTop = (Math.PI * (lat + 1)) / (180 / aDiv);
      var sinThetaTop = Math.sin(thetaTop);
      var cosThetaTop = Math.cos(thetaTop);
      var thetaBot = (Math.PI * (lat - 1)) / (180 / aDiv);
      var sinThetaBot = Math.sin(thetaBot);
      var cosThetaBot = Math.cos(thetaBot);

      for (var lon = 0; lon <= slicePartitions; lon += 1) {
        //Primary vertex calculation
        var phi = (CesiumMath.TWO_PI * lon) / (slicePartitions * (360 / angleAz));
        var sinPhi = Math.sin(phi);
        var cosPhi = Math.cos(phi);

        var x = sinPhi * sinTheta;
        var y = cosPhi * sinTheta;
        var z = cosTheta;

        //Calculations for normals +/-
        var phiLeft = (CesiumMath.TWO_PI * (lon + 1)) / (slicePartitions * (360 / angleAz));
        var sinPhiLeft = Math.sin(phiLeft);
        var cosPhiLeft = Math.cos(phiLeft);
        var phiRight = (CesiumMath.TWO_PI * (lon - 1)) / (slicePartitions * (360 / angleAz));
        var sinPhiRight = Math.sin(phiRight);
        var cosPhiRight = Math.cos(phiRight);

        //Top and bottom vertices
        if (lat === stEl) {
          //Top normals
          var TBxT = x - (sinPhi * sinThetaTop);
          var TByT = y - (cosPhi * sinThetaTop);
          var TBzT = z - cosThetaTop;
          normals[TBindexN++] = (TBxT);
          normals[TBindexN++] = (TByT);
          normals[TBindexN++] = (TBzT);
          normals[TBindexN++] = (TBxT);
          normals[TBindexN++] = (TByT);
          normals[TBindexN++] = (TBzT);
          positions[TBindex++] = (minRange * x);
          positions[TBindex++] = (minRange * y);
          positions[TBindex++] = (minRange * z);
          positions[TBindex++] = (radius * x);
          positions[TBindex++] = (radius * y);
          positions[TBindex++] = (radius * z);
        }
        if (lat === stStop) {
          //Bottom normals
          var TBxB = x - (sinPhi * sinThetaBot);
          var TByB = y - (cosPhi * sinThetaBot);
          var TBzB = z - cosThetaBot;
          normals[TBindexN++] = (TBxB);
          normals[TBindexN++] = (TByB);
          normals[TBindexN++] = (TBzB);
          normals[TBindexN++] = (TBxB);
          normals[TBindexN++] = (TByB);
          normals[TBindexN++] = (TBzB);
          positions[TBindex++] = (minRange * x);
          positions[TBindex++] = (minRange * y);
          positions[TBindex++] = (minRange * z);
          positions[TBindex++] = (radius * x);
          positions[TBindex++] = (radius * y);
          positions[TBindex++] = (radius * z);
        }

        //Left and Right vertices
        if (lon === 0) {
          var LRxL = x - (sinPhiLeft * sinTheta);
          var LRyL = y - (cosPhiLeft * sinTheta);
          var LRzL = z - cosTheta;
          normals[LRindexN++] = (LRxL);
          normals[LRindexN++] = (LRyL);
          normals[LRindexN++] = (LRzL);
          normals[LRindexN++] = (LRxL);
          normals[LRindexN++] = (LRyL);
          normals[LRindexN++] = (LRzL);
          positions[LRindex++] = (minRange * x);
          positions[LRindex++] = (minRange * y);
          positions[LRindex++] = (minRange * z);
          positions[LRindex++] = (radius * x);
          positions[LRindex++] = (radius * y);
          positions[LRindex++] = (radius * z);
        }
        if (lon === slicePartitions) {
          var LRxR = x - (sinPhiRight * sinTheta);
          var LRyR = y - (cosPhiRight * sinTheta);
          var LRzR = z - cosTheta;
          normals[LRindexN++] = (LRxR);
          normals[LRindexN++] = (LRyR);
          normals[LRindexN++] = (LRzR);
          normals[LRindexN++] = (LRxR);
          normals[LRindexN++] = (LRyR);
          normals[LRindexN++] = (LRzR);
          positions[LRindex++] = (minRange * x);
          positions[LRindex++] = (minRange * y);
          positions[LRindex++] = (minRange * z);
          positions[LRindex++] = (radius * x);
          positions[LRindex++] = (radius * y);
          positions[LRindex++] = (radius * z);
        }

        //Inner and outer vertices
        normals[IOindexN++] = (-x);
        normals[IOindexN++] = (-y);
        normals[IOindexN++] = (-z);
        normals[IOindexN++] = (x);
        normals[IOindexN++] = (y);
        normals[IOindexN++] = (z);
        positions[IOindex++] = (minRange * x);
        positions[IOindex++] = (minRange * y);
        positions[IOindex++] = (minRange * z);
        positions[IOindex++] = (radius * x);
        positions[IOindex++] = (radius * y);
        positions[IOindex++] = (radius * z);

      }
    }

    var attributes = new GeometryAttributes();

    if (vertexFormat.position) {
      attributes.position = new GeometryAttribute({
        componentDatatype: ComponentDatatype.DOUBLE,
        componentsPerAttribute: 3,
        values: positions
      });
    }

    if (vertexFormat.normal) {
      attributes.normal = new GeometryAttribute({
        componentDatatype: ComponentDatatype.FLOAT,
        componentsPerAttribute: 3,
        values: normals
      });
    }

    //Inner and Outer indices
    var TBnumIndices = 6 * (slicePartitions * 2);
    TBindex = 0;

    //Left and Right indices
    var LRnumIndices = 6 * (slicePartitions * 2);
    LRindex = TBnumIndices;

    //Inner and Outer indices
    var IOnumIndices = 6 * ((stackPartitions * slicePartitions) * 2);
    IOindex = TBnumIndices + LRnumIndices;

    //Final indices
    var numIndices = TBnumIndices + LRnumIndices + IOnumIndices;
    var indices = IndexDatatype.createTypedArray(vertexCount, numIndices);

    for (var latNum = 0; latNum < (stackPartitions); latNum++) {
      for (var lonNum = 0; lonNum < (slicePartitions * 2); lonNum += 2) {

        //Top and Bottom indices
        if (latNum === 0) {
          var top = lonNum;
          indices[TBindex++] = (top);
          indices[TBindex++] = (top + 1);
          indices[TBindex++] = (top + 2);
          indices[TBindex++] = (top + 2);
          indices[TBindex++] = (top + 1);
          indices[TBindex++] = (top + 3);
        }
        if (latNum === (stackPartitions - 1)) {
          var bottom = lonNum + ((slicePartitions + 1) * 2);
          indices[TBindex++] = (bottom);
          indices[TBindex++] = (bottom + 1);
          indices[TBindex++] = (bottom + 2);
          indices[TBindex++] = (bottom + 2);
          indices[TBindex++] = (bottom + 1);
          indices[TBindex++] = (bottom + 3);
        }

        //Left and Right indices
        if (lonNum === 0) {
          var left = TBvertexCount + (latNum * 4);
          indices[LRindex++] = (left);
          indices[LRindex++] = (left + 1);
          indices[LRindex++] = (left + 4);
          indices[LRindex++] = (left + 4);
          indices[LRindex++] = (left + 1);
          indices[LRindex++] = (left + 5);
          var right = left + 2;
          indices[LRindex++] = (right);
          indices[LRindex++] = (right + 1);
          indices[LRindex++] = (right + 4);
          indices[LRindex++] = (right + 4);
          indices[LRindex++] = (right + 1);
          indices[LRindex++] = (right + 5);
        }

        //Inner and Outer indices
        var level2 = (stackPartitions + 1) * 2;
        var indiceOffset = TBvertexCount + LRvertexCount;
        var inner = indiceOffset + lonNum + (latNum * level2);
        indices[IOindex++] = (inner);
        indices[IOindex++] = (inner + 2);
        indices[IOindex++] = (inner + level2);
        indices[IOindex++] = (inner + 2);
        indices[IOindex++] = (inner + level2);
        indices[IOindex++] = (inner + level2 + 2);
        var outer = inner + 1;
        indices[IOindex++] = (outer);
        indices[IOindex++] = (outer + 2);
        indices[IOindex++] = (outer + level2);
        indices[IOindex++] = (outer + 2);
        indices[IOindex++] = (outer + level2);
        indices[IOindex++] = (outer + level2 + 2);

      }
    }

    return new Geometry({
      attributes: attributes,
      indices: indices,
      primitiveType: PrimitiveType.TRIANGLES,
      boundingSphere: BoundingSphere.fromVertices(positions)
    });
  };

  return HemisphereGeometry;
});
