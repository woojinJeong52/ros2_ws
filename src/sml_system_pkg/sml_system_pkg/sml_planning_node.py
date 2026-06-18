"""
planning_node.py
Task.msg를 수신해서 depends_on 기반 스텝 시퀀스를 생성하고
ManagerNode 요청 시 전달하는 노드.
"""

import copy
import rclpy
from rclpy.node import Node

from sml_msgs.msg import Task, Order, Station, Step
from sml_msgs.srv import GetPlan


PRODUCT_NAMES = {
    34: 'Battery', 13: 'Magnet', 81: 'E-Stop',
    442: 'Carrot', 241: 'Traffic Light', 462: 'Small Tree',
    711: 'Hammer', 4482: 'Big Carrot', 8518: 'Burger',
    48132: 'Ice Cream', 46262: 'Big Tree',
}


class PlanningNode(Node):

    def __init__(self):
        super().__init__('planning_node')

        self.plan_generated = False
        self.steps = []

        self.task_sub = self.create_subscription(
            Task, '/sml/task', self.task_callback, 10
        )
        self.plan_srv = self.create_service(
            GetPlan, '/sml/get_plan', self.get_plan_callback
        )

        self.get_logger().info('PlanningNode 시작')

    # --------------------------------------------------------
    # 콜백
    # --------------------------------------------------------

    def task_callback(self, task):
        if self.plan_generated:
            return
        self.plan_generated = True
        self.get_logger().info('Task 수신 → 계획 생성 시작')
        self._build_plan(task)

    def get_plan_callback(self, request, response):
        if not self.plan_generated or not self.steps:
            response.success = False
            response.message = '계획이 아직 생성되지 않았습니다'
            return response
        response.steps = self.steps
        response.success = True
        response.message = ''
        self.get_logger().info(f'GetPlan 응답: {len(self.steps)}개 스텝 전달')
        return response

    # --------------------------------------------------------
    # 핵심 계획 생성 로직
    # --------------------------------------------------------

    def _build_plan(self, task):
        try:
            station_materials, wb_id, customer_id, storage_id = \
                self._parse_arena(task.arena_layout)

            produce_orders, recycle_orders = \
                self._parse_orders(task.order_list)

            virtual_stock = copy.deepcopy(station_materials)
            recycle_releases = self._register_recycle_releases(recycle_orders)
            self._assign_material_sources(produce_orders, recycle_releases, virtual_stock)
            self._assign_waste_materials(recycle_orders, produce_orders)

            wb_sequence = self._build_wb_sequence(produce_orders, recycle_orders)

            self.steps = self._generate_steps(
                wb_sequence, station_materials,
                wb_id, customer_id, storage_id
            )

            self.get_logger().info(f'계획 생성 완료: {len(self.steps)}개 스텝')
            self._log_plan_summary(produce_orders, recycle_orders)
            self._log_steps(self.steps)

        except Exception as e:
            self.get_logger().error(f'계획 생성 실패: {e}')
            self.plan_generated = False

    # --------------------------------------------------------
    # Step 1: 입력 파싱
    # --------------------------------------------------------

    def _parse_arena(self, arena_layout):
        station_materials = {}
        wb_id = customer_id = storage_id = None

        for station in arena_layout:
            station_materials[station.station_id] = list(station.material_ids)
            if station.station_type == Station.ST_WORKBENCH:
                wb_id = station.station_id
            elif station.station_type == Station.ST_CUSTOMER:
                customer_id = station.station_id
            elif station.station_type == Station.ST_STORAGE and storage_id is None:
                storage_id = station.station_id

        return station_materials, wb_id, customer_id, storage_id

    def _parse_orders(self, order_list):
        produce_orders = []
        recycle_orders = []

        for order in order_list:
            parsed = {
                'order_type':       order.order_type,
                'product_id':       order.product_id,
                'materials':        self._parse_product_id(order.product_id),
                # material_sources: [(material, source, dep_recycle), ...]
                # 리스트로 관리해서 중복 재료도 개수 단위로 처리
                'material_sources': [],
                'waste_materials':  [],
                'reuse_materials':  [],
            }
            if order.order_type == Order.OT_PRODUCE:
                produce_orders.append(parsed)
            elif order.order_type == Order.OT_RECYCLE:
                recycle_orders.append(parsed)

        produce_orders.sort(key=lambda o: len(o['materials']))
        return produce_orders, recycle_orders

    def _parse_product_id(self, product_id):
        return [int(d) for d in str(product_id)]

    # --------------------------------------------------------
    # Step 2: 재료 출처 분류 + 가상 차감
    # --------------------------------------------------------

    def _register_recycle_releases(self, recycle_orders):
        """RECYCLE 후 나올 재료 등록 → {material: [recycle_order, ...]} (개수 단위)"""
        recycle_available = {}
        for order in recycle_orders:
            for material in order['materials']:
                recycle_available.setdefault(material, []).append(order)
        return recycle_available

    def _assign_material_sources(self, produce_orders, recycle_available, virtual_stock):
        """
        재료 출처 결정 + 가상 차감
        우선순위: RECYCLE 후 재료 → 재고
        material_sources = [(material, source, dep_recycle), ...]
        """
        for order in produce_orders:
            for material in order['materials']:

                # 1순위: RECYCLE 후 WB에 생기는 재료
                if material in recycle_available and recycle_available[material]:
                    recycle_order = recycle_available[material].pop(0)
                    order['material_sources'].append((material, 'WB', recycle_order))

                # 2순위: 재고에서 가져오기
                else:
                    station_id = self._find_in_stock(material, virtual_stock)
                    if station_id is not None:
                        virtual_stock[station_id].remove(material)
                        order['material_sources'].append((material, station_id, None))
                    else:
                        raise RuntimeError(f'재료 {material}를 구할 수 없음')

    def _assign_waste_materials(self, recycle_orders, produce_orders):
        """RECYCLE 후 재료 → reuse / waste 분류 (개수 단위)"""
        needed = []
        for order in produce_orders:
            needed.extend(order['materials'])

        remaining_needed = list(needed)
        for order in recycle_orders:
            for material in order['materials']:
                if material in remaining_needed:
                    order['reuse_materials'].append(material)
                    remaining_needed.remove(material)
                else:
                    order['waste_materials'].append(material)

    def _find_in_stock(self, material, stock):
        for station_id, materials in stock.items():
            if material in materials:
                return station_id
        return None

    # --------------------------------------------------------
    # Step 3: WB 시퀀스 결정
    # --------------------------------------------------------

    def _build_wb_sequence(self, produce_orders, recycle_orders):
        """
        1. 재고만으로 가능한 PRODUCE 먼저
        2. RECYCLE 필요한 PRODUCE → 해당 RECYCLE과 묶어서
        3. 독립 RECYCLE (어떤 PRODUCE와도 무관) → 마지막
        4. PRODUCE 결과물 RECYCLE → 맨 마지막
        """
        produce_ids = {o['product_id'] for o in produce_orders}

        # 각 PRODUCE가 의존하는 RECYCLE 파악
        produce_recycle_deps = {}
        for po in produce_orders:
            deps = []
            for (material, source, dep_recycle) in po['material_sources']:
                if dep_recycle is not None and dep_recycle not in deps:
                    deps.append(dep_recycle)
            produce_recycle_deps[id(po)] = deps

        after_recycles      = []
        standalone_recycles = []

        for ro in recycle_orders:
            if ro['product_id'] in produce_ids:
                after_recycles.append(ro)
            else:
                is_linked = any(
                    ro in deps for deps in produce_recycle_deps.values()
                )
                if not is_linked:
                    standalone_recycles.append(ro)

        wb_sequence   = []
        used_recycles = set()

        # 1. 재고만으로 가능한 PRODUCE
        for po in produce_orders:
            if not produce_recycle_deps[id(po)]:
                wb_sequence.append(po)

        # 2. RECYCLE 필요한 PRODUCE → 해당 RECYCLE과 묶음
        for po in produce_orders:
            deps = produce_recycle_deps[id(po)]
            if deps:
                for ro in deps:
                    if id(ro) not in used_recycles:
                        wb_sequence.append(ro)
                        used_recycles.add(id(ro))
                wb_sequence.append(po)

        # 3. 독립 RECYCLE
        wb_sequence.extend(standalone_recycles)

        # 4. PRODUCE 결과물 RECYCLE
        wb_sequence.extend(after_recycles)

        return wb_sequence

    # --------------------------------------------------------
    # Step 4: 스텝 시퀀스 생성
    # --------------------------------------------------------

    def _generate_steps(
        self, wb_sequence, station_materials,
        wb_id, customer_id, storage_id
    ):
        steps = []
        step_id = 0
        last_wb_step_id = None

        slot_1           = None
        slot_material    = []
        pending_loads    = []
        loaded_materials = set()  # 중복 적재 방지

        for wb_task in wb_sequence:

            # ------------------------------------------------
            # RECYCLE: 분해 대상 슬롯 1에 Load
            # ------------------------------------------------
            if wb_task['order_type'] == Order.OT_RECYCLE:

                if slot_1 is not None:
                    step_id, last_wb_step_id = self._flush_unload(
                        steps, step_id, pending_loads,
                        slot_1, slot_material, wb_id, last_wb_step_id
                    )
                    slot_1        = None
                    slot_material = []
                    pending_loads = []

                src = self._find_in_stock(wb_task['product_id'], station_materials)
                steps.append(self._make_step(
                    step_id, Step.AMR, Step.LOAD,
                    [wb_task['product_id']], src, []
                ))
                pending_loads.append(step_id)
                slot_1 = wb_task['product_id']
                step_id += 1

                # 다음 PRODUCE 재료 미리 적재 (재고에서 가져올 수 있는 것만)
                preload_by_station = {}
                for future_task in wb_sequence:
                    if future_task['order_type'] != Order.OT_PRODUCE:
                        continue
                    for (material, source, dep) in future_task['material_sources']:
                        if dep is None \
                                and len(slot_material) < 5 \
                                and material not in loaded_materials:
                            preload_by_station.setdefault(source, []).append(material)
                            slot_material.append(material)
                            loaded_materials.add(material)

                for source, materials in preload_by_station.items():
                    steps.append(self._make_step(
                        step_id, Step.AMR, Step.LOAD,
                        materials, source, []
                    ))
                    pending_loads.append(step_id)
                    step_id += 1

            # ------------------------------------------------
            # PRODUCE: 재료 Load
            # ------------------------------------------------
            elif wb_task['order_type'] == Order.OT_PRODUCE:

                needs_wb_material = any(
                    dep is not None
                    for (_, _, dep) in wb_task['material_sources']
                )

                if needs_wb_material and pending_loads:
                    step_id, last_wb_step_id = self._flush_unload(
                        steps, step_id, pending_loads,
                        slot_1, slot_material, wb_id, last_wb_step_id
                    )
                    slot_1        = None
                    slot_material = []
                    pending_loads = []

                load_by_station = {}
                for (material, source, dep) in wb_task['material_sources']:
                    if dep is None and material not in loaded_materials:
                        load_by_station.setdefault(source, []).append(material)
                        slot_material.append(material)
                        loaded_materials.add(material)

                for source, materials in load_by_station.items():
                    steps.append(self._make_step(
                        step_id, Step.AMR, Step.LOAD,
                        materials, source, []
                    ))
                    pending_loads.append(step_id)
                    step_id += 1

            # ------------------------------------------------
            # WB Unload
            # ------------------------------------------------
            all_objects = (
                ([slot_1] if slot_1 is not None else []) + slot_material
            )

            if all_objects:
                unload_depends = list(pending_loads)
                if last_wb_step_id is not None:
                    unload_depends.append(last_wb_step_id)

                unload_step_id = step_id
                steps.append(self._make_step(
                    step_id, Step.AMR, Step.UNLOAD,
                    all_objects, wb_id, unload_depends
                ))
                step_id += 1
            else:
                unload_step_id = None

            # ------------------------------------------------
            # WB 작업 (PRODUCE or RECYCLE)
            # ------------------------------------------------
            wb_action  = Step.PRODUCE \
                if wb_task['order_type'] == Order.OT_PRODUCE else Step.RECYCLE
            # PRODUCE: 완성품 ID / RECYCLE: 분해 대상 ID
            wb_objects = [wb_task['product_id']]

            wb_depends = []
            if unload_step_id is not None:
                wb_depends.append(unload_step_id)
            if last_wb_step_id is not None:
                wb_depends.append(last_wb_step_id)

            if wb_task['order_type'] == Order.OT_PRODUCE:
                for (_, _, dep_recycle) in wb_task['material_sources']:
                    if dep_recycle is not None:
                        recycle_sid = self._find_wb_recycle_step_id(
                            steps, dep_recycle['product_id']
                        )
                        if recycle_sid is not None and recycle_sid not in wb_depends:
                            wb_depends.append(recycle_sid)

            steps.append(self._make_step(
                step_id, Step.WB, wb_action,
                wb_objects, wb_id, wb_depends
            ))
            last_wb_step_id = step_id
            step_id += 1

            # ------------------------------------------------
            # PRODUCE 완료 후: 완성품 납품
            # ------------------------------------------------
            if wb_task['order_type'] == Order.OT_PRODUCE:
                load_sid = step_id
                steps.append(self._make_step(
                    step_id, Step.AMR, Step.LOAD,
                    [wb_task['product_id']], wb_id, [last_wb_step_id]
                ))
                step_id += 1
                steps.append(self._make_step(
                    step_id, Step.AMR, Step.UNLOAD,
                    [wb_task['product_id']], customer_id, [load_sid]
                ))
                step_id += 1

            # ------------------------------------------------
            # RECYCLE 완료 후: 불필요 재료 Storage 반납
            # ------------------------------------------------
            if wb_task['order_type'] == Order.OT_RECYCLE \
                    and wb_task['waste_materials']:
                load_sid = step_id
                steps.append(self._make_step(
                    step_id, Step.AMR, Step.LOAD,
                    wb_task['waste_materials'], wb_id, [last_wb_step_id]
                ))
                step_id += 1
                steps.append(self._make_step(
                    step_id, Step.AMR, Step.UNLOAD,
                    wb_task['waste_materials'], storage_id, [load_sid]
                ))
                step_id += 1

            slot_1        = None
            slot_material = []
            pending_loads = []

        return steps

    # --------------------------------------------------------
    # 헬퍼 함수
    # --------------------------------------------------------

    def _make_step(self, step_id, type_, action, object_ids, station_id, depends_on):
        step = Step()
        step.step_id    = step_id
        step.type       = type_
        step.action     = action
        step.object_ids = list(object_ids)
        step.station_id = station_id if station_id is not None else -1
        step.depends_on = list(depends_on)
        return step

    def _flush_unload(
        self, steps, step_id, pending_loads,
        slot_1, slot_material, wb_id, last_wb_step_id
    ):
        unload_depends = list(pending_loads)
        if last_wb_step_id is not None:
            unload_depends.append(last_wb_step_id)
        all_objects = (([slot_1] if slot_1 is not None else []) + slot_material)
        steps.append(self._make_step(
            step_id, Step.AMR, Step.UNLOAD,
            all_objects, wb_id, unload_depends
        ))
        return step_id + 1, last_wb_step_id

    def _find_wb_recycle_step_id(self, steps, product_id):
        for step in steps:
            if step.type == Step.WB and step.action == Step.RECYCLE:
                if product_id in step.object_ids:
                    return step.step_id
        return None

    # --------------------------------------------------------
    # 로그
    # --------------------------------------------------------

    def _log_plan_summary(self, produce_orders, recycle_orders):
        def name(pid):
            return PRODUCT_NAMES.get(pid, str(pid))

        self.get_logger().info('===== 실행 계획 요약 =====')

        for order in recycle_orders:
            pid = order['product_id']
            self.get_logger().info(f'[RECYCLE] {pid} ({name(pid)})')
            self.get_logger().info(f'  -> 분해 후: {order["materials"]}')
            if order['reuse_materials']:
                # reuse 재료별로 어느 PRODUCE에 사용되는지 표시
                # material_sources가 리스트라 중복 재료도 순서대로 매핑 가능
                reuse_info = []
                reuse_remaining = list(order['reuse_materials'])
                for po in produce_orders:
                    for (material, source, dep_recycle) in po['material_sources']:
                        if dep_recycle is order and material in reuse_remaining:
                            reuse_remaining.remove(material)
                            reuse_info.append(
                                f'{material} -> PRODUCE {po["product_id"]}'
                            )
                self.get_logger().info(
                    f'  -> reuse : {order["reuse_materials"]}  ({" / ".join(reuse_info)})'
                )
            if order['waste_materials']:
                self.get_logger().info(
                    f'  -> waste : {order["waste_materials"]}  -> Storage 반납'
                )

        for order in produce_orders:
            pid = order['product_id']
            self.get_logger().info(f'[PRODUCE] {pid} ({name(pid)})')
            self.get_logger().info(f'  -> 재료: {order["materials"]}')
            for (material, source, dep_recycle) in order['material_sources']:
                if dep_recycle is not None:
                    self.get_logger().info(
                        f'  -> {material} : RECYCLE {dep_recycle["product_id"]} 후 WB에서 재사용'
                    )
                else:
                    self.get_logger().info(
                        f'  -> {material} : station={source} 에서 Load'
                    )

        self.get_logger().info('==========================')

    def _log_steps(self, steps):
        type_map   = {Step.AMR: 'AMR', Step.WB: 'WB '}
        action_map = {
            Step.LOAD:    'LOAD   ',
            Step.UNLOAD:  'UNLOAD ',
            Step.PRODUCE: 'PRODUCE',
            Step.RECYCLE: 'RECYCLE',
        }
        self.get_logger().info('===== 스텝 시퀀스 =====')
        for s in steps:
            self.get_logger().info(
                f'[{s.step_id:2d}] {type_map[s.type]} | '
                f'{action_map[s.action]} | '
                f'objects={list(s.object_ids)} | '
                f'station={s.station_id} | '
                f'depends_on={list(s.depends_on)}'
            )
        self.get_logger().info('======================')


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
