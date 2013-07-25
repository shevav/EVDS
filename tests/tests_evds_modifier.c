#include "framework.h"

void Test_EVDS_MODIFIER() {
	START_TEST("Linear modifier test") {

/*<EVDS version="31">
    <object name="Modifier" type="modifier">
        <parameter name="pattern">linear</parameter>
        <parameter name="vector1.count">5</parameter>
        <parameter name="vector2.count">5</parameter>
        <parameter name="vector1.x">2</parameter>
        <parameter name="vector2.x">0</parameter>
        <parameter name="vector2.y">2</parameter>
        <parameter name="vector3.count">0</parameter>
        <object name="New object 2" type="static_body">
            <parameter name="geometry.cross_sections">
                <section type="ellipse" />
                <section type="ellipse" rx="1" ry="1" />
            </parameter>
            <parameter name="mass">100</parameter>
        </object>
    </object>
    <object type="metadata">
        <object type="foxworks.schematics" />
    </object>
</EVDS>*/
	} END_TEST
}