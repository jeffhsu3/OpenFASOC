from gdsfactory.typings import Component
from pydantic import validate_arguments


@validate_arguments
def component_snap_to_grid(comp: Component) -> Component:
	"""snaps all polygons and ports in component to grid
	comp = the component to snap to grid
	NOTE this function will flatten the component
	NOTE labels are cell-local: only the top cell's own labels survive. flatten()
	pulls every subcell instance's labels (transformed) into the top cell, and
	magic unions same-named labels within one cell into one node -- so duplicate
	texts from repeated child instances short nets BY NAME with no metal involved
	(this silently poisoned differential_delay_stage: VINP+VINN merged by the two
	sample_hold instances' internal "VIN" labels).
	"""
	# flatten the component then copy (the copy snaps polygons and ports to grid)
	name = comp.name
	# snapshot the top cell's OWN labels (gdstk Cell.labels excludes children's)
	own = {(l.text, float(l.origin[0]), float(l.origin[1])) for l in comp.labels}
	flat = comp.flatten()
	# strip BETWEEN flatten() and .copy(): the copy is what snaps coordinates, so
	# here the top cell's own labels still carry their exact pre-flatten origins
	# and an exact (text, x, y) match is safe. Inherited labels arrive transformed
	# and won't match; one landing exactly on an own label is harmless (same name,
	# same spot).
	inherited = [l for l in flat.labels
	             if (l.text, float(l.origin[0]), float(l.origin[1])) not in own]
	if inherited:
		# gdstk Cell.labels returns a FRESH list per access -- list.remove() is a
		# silent no-op; removal must go through Cell.remove()
		flat._cell.remove(*inherited)
	comp = flat.copy()
	comp.name = name
	return comp


