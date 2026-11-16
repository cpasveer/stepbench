// Line icons for the toolbar, drawn here: 20 x 20 grid, one stroke weight
// (1.5), round joins. Body icons have three parts so the button shows its
// state: .fill (solid: filled; ghost: faint; hidden: none), .edge (dashed
// when ghosted) and .slash (only when hidden).

const svg = (inner) => `<svg viewBox="0 0 20 20" aria-hidden="true" focusable="false">${inner}</svg>`;

const body = (shape, extra = '') => svg(
  `<path class="fill" d="${shape}"/><path class="edge" d="${shape}"/>${extra}<path class="slash" d="M3 17 L17 3"/>`);

export const ICONS = {
  // a block: the fixed parts
  fixed: body('M10 2.5 L16.5 6 V14 L10 17.5 L3.5 14 V6 Z', '<path class="edge" d="M3.5 6 L10 9.5 L16.5 6 M10 9.5 V17.5"/>'),
  // a bracket: the answer
  part: body('M4 3.5 H8 V12 H16 V16.5 H4 Z'),
  // the bracket with a tick: the reference answer
  ref: body('M4 3.5 H8 V12 H16 V16.5 H4 Z', '<path class="edge" d="M11 6 L13 8 L17 3.5"/>'),
  // a keep-out block, cross-braced: obstacles (level 3)
  obstacle: body('M3.5 4 H16.5 V16 H3.5 Z', '<path class="edge" d="M3.5 4 L16.5 16 M16.5 4 L3.5 16"/>'),
  // a flat face with its normal: mounting faces
  faces: svg('<path class="edge" d="M2 15 L7 10 H18 L13 15 Z"/><path class="edge" d="M10 12.5 V3 M8 5 L10 3 L12 5"/>'),
  // a socket head screw: bolts
  bolts: svg('<path class="edge" d="M6 2.5 H14 V7 H6 Z M8 7 V17.5 H12 V7 M8 10.5 L12 9.5 M8 13.5 L12 12.5"/>'),
  // scissors: section
  section: svg('<circle class="edge" cx="6" cy="15" r="2.5"/><circle class="edge" cx="14" cy="15" r="2.5"/><path class="edge" d="M7.6 13.1 L15 2.5 M12.4 13.1 L5 2.5"/>'),
  // four corners round a frame: show the whole model
  fit: svg('<path class="edge" d="M3 7 V3 H7 M13 3 H17 V7 M17 13 V17 H13 M7 17 H3 V13"/><rect class="edge" x="7" y="7" width="6" height="6" rx="1"/>'),
  // a lens: zoom (press and drag)
  zoom: svg('<circle class="edge" cx="8.5" cy="8.5" r="5"/><path class="edge" d="M12.2 12.2 L17 17"/>'),
  // a cross of arrows: pan (press and drag)
  pan: svg('<path class="edge" d="M10 2.5 V17.5 M2.5 10 H17.5 M8 4.5 L10 2.5 L12 4.5 M8 15.5 L10 17.5 L12 15.5 M4.5 8 L2.5 10 L4.5 12 M15.5 8 L17.5 10 L15.5 12"/>'),
  // a block drawn in parallel lines: parallel projection on/off
  projection: svg('<path class="edge" d="M4 7 L8 4 H16 V12 L12 15 H4 Z M4 7 H12 V15 M12 7 L16 4"/>'),
  // a box with dashed faces: clipping box
  clipbox: svg('<path class="edge" d="M3.5 6.5 H13.5 V16.5 H3.5 Z" stroke-dasharray="2 1.6"/><path class="edge" d="M3.5 6.5 L6.5 3.5 H16.5 V13.5 L13.5 16.5 M13.5 6.5 L16.5 3.5"/>'),
  // a face with a pointer: clip on a face
  onface: svg('<path class="edge" d="M2 14 L7 9 H17 L12 14 Z"/><path class="edge" d="M10.5 2.5 L10.5 10 L12.5 8.2 L14 11.5"/>'),
  // a ruler: measure
  measure: svg('<path class="edge" d="M2.5 13 L13 2.5 L17.5 7 L7 17.5 Z M6 9.5 L7.8 11.3 M8.5 7 L10.3 8.8 M11 4.5 L12.8 6.3"/>'),
  // a cross: close
  close: svg('<path class="edge" d="M5 5 L15 15 M15 5 L5 15"/>'),
  // flip: two arrows
  flip: svg('<path class="edge" d="M4 7 H15 M12 4 L15 7 L12 10 M16 13 H5 M8 10 L5 13 L8 16"/>'),
  // an arrow into a tray: download
  download: svg('<path class="edge" d="M10 2.5 V12.5 M6 8.5 L10 12.5 L14 8.5 M3.5 13.5 V17 H16.5 V13.5"/>'),
};
