do
        local nimbro_proto = Proto("nimbro_topic", "nimbro_topic_transport");

        nimbro_proto.prefs.port   = Pref.uint( "nimbro_port", 5000, "UDP port" )

        nimbro_proto.fields.packet = ProtoField.uint24("nimbro.packet", "Packet ID", base.DEC)
        nimbro_proto.fields.msg   = ProtoField.uint16("nimbro.msg", "Message ID", base.DEC)
        nimbro_proto.fields.symbol   = ProtoField.uint24("nimbro.symbol", "Symbol ID", base.DEC)

        nimbro_proto.fields.source_symbols   = ProtoField.uint16("nimbro.source_symbols", "Source symbols", base.DEC)
        nimbro_proto.fields.repair_symbols   = ProtoField.uint16("nimbro.repair_symbols", "Repair Symbols", base.DEC)
        nimbro_proto.fields.seed   = ProtoField.uint16("nimbro.seed", "Seed", base.DEC)

        nimbro_proto.fields.topic   = ProtoField.string("nimbro.topic", "Topic", base.ASCII)
        nimbro_proto.fields.type   = ProtoField.string("nimbro.type", "Type", base.ASCII)

        local prev_proto
        local f_udp    = Field.new("udp")

        function nimbro_proto.dissector(tvb, pinfo, tree)
            pcall(function()prev_proto:call(tvb, pinfo, tree)end)

            if not f_udp() then return end


            -- this is just to add text to "nimbro.data" field, 
            -- which you should display as column.
            -- as an alternate, you may remove set_hidden() and view selected data in the treeview
            tree:add_le(nimbro_proto.fields.packet, tvb(0,3));
            tree:add_le(nimbro_proto.fields.msg, tvb(3,2));
            tree:add_le(nimbro_proto.fields.symbol, tvb(5,3));

            tree:add_le(nimbro_proto.fields.source_symbols, tvb(8,2));
            tree:add_le(nimbro_proto.fields.repair_symbols, tvb(10,2));
            tree:add_le(nimbro_proto.fields.seed, tvb(12,4));

            if tvb(5,3):le_uint() == 0 then
				tree:add(nimbro_proto.fields.topic, tvb(16+0,64));
				tree:add(nimbro_proto.fields.type, tvb(16+64,64));
			end
        end

        -- if we hook upon UDP port, then offset will mean the beginning of the UDP data
        udp_table = DissectorTable.get("udp.port")
        prev_proto = udp_table:get_dissector(nimbro_proto.prefs.port)
        udp_table:add(nimbro_proto.prefs.port, nimbro_proto)

        -- if we hook as post dissector, the offset will be from start of the frame. 
        -- don't forget to remove the prev_proto call if you'll use that kind of hook
--        register_postdissector(nimbro_proto)
end
