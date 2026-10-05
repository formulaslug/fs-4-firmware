import re
import os

def parse_canbus_h(filename):
    with open(filename, 'r') as f:
        content = f.read()

    # typedef PREPACK struct {
    # 	int16_t SME_THROTL_TorqueDemand; /* scaling 1.0, offset 0.0, units Q15  */
    #   ...
    # } POSTPACK can_0x186_SME_RPDO_Throttle_Demand_t;
    struct_pattern = re.compile(r'typedef PREPACK struct \{(.*?)\} POSTPACK (can_0x([0-9a-fA-F]+)_(\w+)_t);', re.DOTALL)
    
    messages = []
    
    for match in struct_pattern.finditer(content):
        body = match.group(1)
        full_type_name = match.group(2) # e.g. can_0x186_SME_RPDO_Throttle_Demand_t
        can_id_hex = match.group(3)
        msg_name_suffix = match.group(4) # e.g. SME_RPDO_Throttle_Demand
        
        # Parse fields
        # type name; /* comments */
        field_pattern = re.compile(r'\s*([a-zA-Z0-9_]+)\s+([a-zA-Z0-9_]+);\s*/\*.*?\*/')
        fields = []
        for fmatch in field_pattern.finditer(body):
            fields.append({
                'type': fmatch.group(1),
                'name': fmatch.group(2)
            })
            
        messages.append({
            'type_name': full_type_name,
            'id': int(can_id_hex, 16),
            'name_suffix': msg_name_suffix,
            'struct_member_name': full_type_name[:-2], # remove _t, e.g. can_0x186_SME_RPDO_Throttle_Demand
            'fields': fields
        })
        
    return messages

def generate_header(messages, output_filename):
    code = """#ifndef CAN_GENERATED_H
#define CAN_GENERATED_H

#include "CANbus.h"
#include "mbed.h"
#include <cstring>

"""
    for msg in messages:
        # Extract SME_RPDO_Throttle_Demand)
        class_name = msg['name_suffix']
        
        code += f"struct {class_name} {{\n"
        
        # Members
        for field in msg['fields']:
            code += f"    {field['type']} {field['name']} = 0;\n"
            
        code += "\n"
        
        # Encode
        code += f"    CANMessage encode() {{\n"
        code += f"        can_obj_canbus_h_t obj;\n"
        code += f"        memset(&obj, 0, sizeof(obj));\n"
        
        for field in msg['fields']:
            code += f"        obj.{msg['struct_member_name']}.{field['name']} = {field['name']};\n"
            
        code += f"\n"
        code += f"        uint64_t data = 0;\n"
        code += f"        int dlc = pack_message(&obj, 0x{msg['id'] :X}, &data);\n"
        code += f"        return CANMessage(0x{msg['id'] :X}, (const char*)&data, (uint8_t)dlc);\n"
        code += f"    }}\n\n"
        
        # Decode
        code += f"    bool decode(const CANMessage& msg) {{\n"
        code += f"        if (msg.id != 0x{msg['id'] :X}) return false;\n"
        code += f"        can_obj_canbus_h_t obj;\n"
        code += f"        uint64_t data = 0;\n"
        # data is a sequence of bytes, we need to align the bytes to avoid undefined behavior
        code += f"        memcpy(&data, msg.data, (msg.len > 8) ? 8 : msg.len);\n"
        code += f"\n"
        # We need to call unpack_message. 
        # unpack_message(can_obj_canbus_h_t *o, const unsigned long id, uint64_t data, uint8_t dlc, dbcc_time_stamp_t time_stamp);
        code += f"        if (unpack_message(&obj, 0x{msg['id'] :X}, data, msg.len, 0) < 0) return false;\n"
        code += f"\n"
        for field in msg['fields']:
            code += f"        {field['name']} = obj.{msg['struct_member_name']}.{field['name']};\n"
        code += f"        return true;\n"
        code += f"    }}\n"

        code += f"}};\n\n"

    code += "#endif // CAN_GENERATED_H\n"
    
    with open(output_filename, 'w') as f:
        f.write(code)
    
    print(f"Generated {output_filename} with {len(messages)} classes.")

import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate CAN C++ helpers')
    parser.add_argument('input_header', help='Path to CANbus.h')
    parser.add_argument('output_header', help='Path to output CanGenerated.h')
    args = parser.parse_args()

    msgs = parse_canbus_h(args.input_header)
    generate_header(msgs, args.output_header)
