---
description: "Patrones avanzados de formularios usando JSON Patch. Explicamos cómo construir implementaciones de formularios a nivel empresarial con RxJS y TypeScript, incluyendo autoguardado y Undo/Redo para formularios a gran escala, sincronización en tiempo real para edición colaborativa, soporte offline y seguimiento del historial de operaciones."
---

# Patrones Avanzados de Formularios usando JSON Patch

Al implementar formularios a gran escala o edición colaborativa en tiempo real, el enfoque tradicional de "enviar el formulario completo" genera problemas de rendimiento y experiencia de usuario.

Este artículo explica patrones avanzados de formularios usando **JSON Patch (RFC 6902)**. Al enviar solo las diferencias, se reduce el ancho de banda de red y se puede implementar eficientemente Undo/Redo y edición colaborativa.

## Qué aprenderás en este artículo

- Fundamentos de JSON Patch/Pointer (RFC 6902/6901)
- Autoguardado en formularios a gran escala (basado en diferencias)
- Implementación de Undo/Redo (parches inversos)
- Sincronización en tiempo real para edición colaborativa
- Fundamentos de Operational Transform (OT) / CRDT
- Patrones de integración de WebSocket y RxJS
- Resolución de conflictos y gestión de versiones

> [!TIP] Conocimientos previos
> Este artículo asume conocimiento de [Capítulo 4: Operadores](../operators/index.md), [Patrones de manejo de formularios](./form-handling.md), y [Procesamiento de datos en tiempo real](./real-time-data.md).

> [!NOTE] Cuándo necesitas este patrón
> - **Formularios a gran escala** (más de 100 campos) que requieren autoguardado
> - Funcionalidad **Undo/Redo** es esencial
> - **Edición colaborativa en tiempo real** (funcionalidad similar a Google Docs)
> - **Soporte offline** con encolamiento de diferencias necesario
>
> Para formularios pequeños (~20 campos), los [patrones normales de manejo de formularios](./form-handling.md) son suficientes.

## Fundamentos de JSON Patch/Pointer

### Qué es JSON Patch

**JSON Patch (RFC 6902)** es un formato estándar para expresar cambios en documentos JSON. Puedes enviar solo **el contenido modificado** en lugar del formulario completo.

```typescript
// Datos del formulario antes del cambio
const before = {
  profile: {
    name: "Taro Tanaka",
    email: "tanaka@example.com",
    age: 30
  }
};

// Datos del formulario después del cambio
const after = {
  profile: {
    name: "Taro Tanaka",
    email: "tanaka.updated@example.com", // modificado
    age: 31 // modificado
  }
};

// JSON Patch (diferencia)
const patch = [
  { op: "replace", path: "/profile/email", value: "tanaka.updated@example.com" },
  { op: "replace", path: "/profile/age", value: 31 }
];
```

> [!NOTE] Las 6 operaciones de JSON Patch
> - `add` - Agregar un valor
> - `remove` - Eliminar un valor
> - `replace` - Reemplazar un valor
> - `move` - Mover un valor
> - `copy` - Copiar un valor
> - `test` - Probar un valor (validación)

### Qué es JSON Pointer

**JSON Pointer (RFC 6901)** es una notación de ruta para apuntar a valores específicos dentro de un documento JSON.

```typescript
const formData = {
  user: {
    profile: {
      name: "Taro Tanaka"
    },
    settings: {
      notifications: true
    }
  }
};

// Ejemplos de JSON Pointer
"/user/profile/name"           // → "Taro Tanaka"
"/user/settings/notifications" // → true
"/user/profile"                // → { name: "Taro Tanaka" }
```

### Detección de diferencias con RxJS

Combina `pairwise()` con la librería `fast-json-patch` para detectar automáticamente los cambios en el formulario.

```typescript
import { BehaviorSubject, pairwise, map } from 'rxjs';
import { compare } from 'fast-json-patch';

interface FormData {
  profile: {
    name: string;
    email: string;
    age: number;
  };
}

const initialData: FormData = {
  profile: {
    name: "",
    email: "",
    age: 0
  }
};

const formData$ = new BehaviorSubject<FormData>(initialData);

// Detectar diferencias
const patches$ = formData$.pipe(
  pairwise(), // Obtener el par [valor anterior, valor actual]
  map(([previous, current]) => compare(previous, current))
);

patches$.subscribe(patches => {
  console.log('Cambios detectados:', patches);
  // Ejemplo: [{ op: "replace", path: "/profile/name", value: "Taro Tanaka" }]
});

// Simular actualización de formulario
formData$.next({
  profile: {
    name: "Taro Tanaka",
    email: "tanaka@example.com",
    age: 30
  }
});
```

> [!TIP] Librería fast-json-patch
> ```bash
> npm install fast-json-patch
> ```
> - Cumplimiento completo de RFC 6902
> - Generación de diferencias (`compare`) y aplicación (`applyPatch`)
> - Generación de parches inversos (para Undo)
> - Soporte TypeScript

---

## Autoguardado y Undo/Redo en formularios a gran escala

Implementamos funcionalidad de autoguardado y Undo/Redo para formularios a gran escala (por ejemplo: registro de miembros con 100 campos, pantalla de gestión de productos).

### Enfoque de implementación

**Responsabilidades del Frontend:**
- Generación y secuenciación de operaciones (cambios)
- Reflexión optimista de UI (`scan` para aplicación inmediata)
- Gestión de pilas Undo/Redo (historial mediante parches inversos)
- Gestión de cola de envío (garantía de orden con `concatMap`)
- Agrupación en lotes (`bufferTime` + compresión)

**Responsabilidades del Backend:**
- Gestión de versiones (Vector Clock / timestamp)
- Garantía de idempotencia (detección de duplicados mediante Request ID)
- Persistencia y registro de auditoría

### Patrón 1: Autoguardado básico

Detecta cambios en el formulario y envía lotes al servidor a intervalos regulares.

```typescript
import {
  BehaviorSubject,
  Subject,
  pairwise,
  map,
  bufferTime,
  filter,
  concatMap,
  catchError,
  of
} from 'rxjs';
import { compare, Operation } from 'fast-json-patch';

interface LargeFormData {
  personalInfo: {
    firstName: string;
    lastName: string;
    email: string;
    phone: string;
    dateOfBirth: string;
  };
  address: {
    street: string;
    city: string;
    state: string;
    zipCode: string;
    country: string;
  };
  preferences: {
    newsletter: boolean;
    notifications: boolean;
    theme: 'light' | 'dark';
  };
  // ... Asumiendo más de 100 campos
}

const initialFormData: LargeFormData = {
  personalInfo: {
    firstName: "",
    lastName: "",
    email: "",
    phone: "",
    dateOfBirth: ""
  },
  address: {
    street: "",
    city: "",
    state: "",
    zipCode: "",
    country: "Japan"
  },
  preferences: {
    newsletter: false,
    notifications: true,
    theme: 'light'
  }
};

// Stream de datos del formulario
const formData$ = new BehaviorSubject<LargeFormData>(initialFormData);

// Stream de resultados de guardado
const saveResult$ = new Subject<{ success: boolean; message: string }>();

// Elemento de visualización de estado para demostración
const statusDiv = document.createElement('div');
statusDiv.style.padding = '10px';
statusDiv.style.margin = '10px';
statusDiv.style.border = '2px solid #ccc';
statusDiv.style.borderRadius = '4px';
statusDiv.style.fontFamily = 'monospace';
statusDiv.style.fontSize = '14px';
document.body.appendChild(statusDiv);

function updateStatus(message: string, color: string = '#333') {
  statusDiv.innerHTML = `<span style="color: ${color}">${message}</span>`;
}

// Pipeline de autoguardado
formData$.pipe(
  pairwise(),
  map(([previous, current]) => ({
    patches: compare(previous, current),
    timestamp: Date.now()
  })),
  filter(({ patches }) => patches.length > 0), // Omitir si no hay cambios
  bufferTime(2000), // Almacenar en buffer los cambios durante 2 segundos
  filter(buffer => buffer.length > 0), // Omitir buffers vacíos
  map(buffer => {
    // Consolidar todos los parches del buffer en un solo array
    const allPatches = buffer.flatMap(item => item.patches);
    updateStatus(`📦 Procesando ${allPatches.length} cambios en lote...`, '#FF9800');
    return allPatches;
  }),
  concatMap(patches => saveToServer(patches)), // Enviar con orden garantizado
  catchError((error: unknown) => {
    console.error('Error de autoguardado:', error);
    updateStatus(`❌ Fallo al guardar: ${(error instanceof Error ? error.message : String(error))}`, '#f44336');
    return of({ success: false, message: (error instanceof Error ? error.message : String(error)) });
  })
).subscribe(result => {
  if (result.success) {
    updateStatus(`✅ Autoguardado completado (${new Date().toLocaleTimeString()})`, '#4CAF50');
  }
  saveResult$.next(result);
});

// Guardar en servidor (implementación mock)
function saveToServer(patches: Operation[]): Promise<{ success: boolean; message: string }> {
  console.log('Enviando al servidor:', patches);

  // Ejemplo de implementación real:
  // return fetch('/api/forms/12345/patches', {
  //   method: 'PATCH',
  //   headers: { 'Content-Type': 'application/json-patch+json' },
  //   body: JSON.stringify(patches)
  // }).then(res => res.json());

  // Mock: devolver éxito después de 500ms
  return new Promise(resolve => {
    setTimeout(() => {
      resolve({
        success: true,
        message: `Guardados ${patches.length} cambios`
      });
    }, 500);
  });
}

// Demo: Simular cambio de formulario
const demoButton = document.createElement('button');
demoButton.textContent = 'Cambiar formulario (Demo)';
demoButton.style.padding = '10px 20px';
demoButton.style.margin = '10px';
demoButton.style.fontSize = '16px';
demoButton.style.cursor = 'pointer';
document.body.appendChild(demoButton);

demoButton.addEventListener('click', () => {
  // Cambiar campos aleatoriamente
  const currentData = formData$.getValue();
  const updatedData = {
    ...currentData,
    personalInfo: {
      ...currentData.personalInfo,
      firstName: `Taro_${Math.floor(Math.random() * 100)}`,
      email: `taro${Math.floor(Math.random() * 100)}@example.com`
    },
    preferences: {
      ...currentData.preferences,
      newsletter: !currentData.preferences.newsletter
    }
  };
  formData$.next(updatedData);
  updateStatus('📝 Formulario modificado...', '#2196F3');
});
```

> [!NOTE] Puntos clave del autoguardado
> - **`bufferTime(2000)`** - Agrupar cambios de 2 segundos para enviar juntos (eficiencia de red)
> - **`concatMap`** - Garantizar el orden de los parches (`mergeMap` puede alterar el orden)
> - **`filter`** - Omitir cuando no hay cambios (reducir solicitudes innecesarias)
> - **Idempotencia** - Seguro enviar el mismo parche varias veces (agregar Request ID)

### Patrón 2: Implementación de Undo/Redo

Implementa funcionalidad Undo/Redo usando parches inversos.

```typescript
import { Subject, scan } from 'rxjs';
import { applyPatch, Operation, deepClone } from 'fast-json-patch';

interface HistoryState<T> {
  current: T;
  undoStack: Operation[][];
  redoStack: Operation[][];
}

interface HistoryAction {
  type: 'APPLY_PATCH' | 'UNDO' | 'REDO';
  patches?: Operation[];
}

// Stream de gestión de historial
const historyAction$ = new Subject<HistoryAction>();

const initialState: HistoryState<LargeFormData> = {
  current: initialFormData,
  undoStack: [],
  redoStack: []
};

// Reducer para gestionar el historial
const history$ = historyAction$.pipe(
  scan((state, action) => {
    switch (action.type) {
      case 'APPLY_PATCH':
        if (!action.patches || action.patches.length === 0) return state;

        // Aplicar parche
        const cloned = deepClone(state.current);
        const result = applyPatch(cloned, action.patches, true, false);

        return {
          current: result.newDocument,
          undoStack: [...state.undoStack, action.patches],
          redoStack: [] // Limpiar pila de Redo con nueva operación
        };

      case 'UNDO':
        if (state.undoStack.length === 0) return state;

        const patchesToUndo = state.undoStack[state.undoStack.length - 1];
        const beforeUndo = deepClone(state.current);

        // Generar y aplicar parche inverso
        const inversePatch = generateInversePatch(state.current, patchesToUndo);
        const undoResult = applyPatch(beforeUndo, inversePatch, true, false);

        return {
          current: undoResult.newDocument,
          undoStack: state.undoStack.slice(0, -1),
          redoStack: [...state.redoStack, patchesToUndo]
        };

      case 'REDO':
        if (state.redoStack.length === 0) return state;

        const patchesToRedo = state.redoStack[state.redoStack.length - 1];
        const beforeRedo = deepClone(state.current);
        const redoResult = applyPatch(beforeRedo, patchesToRedo, true, false);

        return {
          current: redoResult.newDocument,
          undoStack: [...state.undoStack, patchesToRedo],
          redoStack: state.redoStack.slice(0, -1)
        };

      default:
        return state;
    }
  }, initialState)
);

// Generación de parche inverso (implementación simplificada)
function generateInversePatch(document: any, patches: Operation[]): Operation[] {
  // El applyPatch de fast-json-patch devuelve el parche inverso cuando el 4to argumento es true
  const cloned = deepClone(document);
  const result = applyPatch(cloned, patches, true, true);
  return result[1] || []; // Obtener parche inverso
}

// Elementos de UI
const historyControlDiv = document.createElement('div');
historyControlDiv.style.padding = '10px';
historyControlDiv.style.margin = '10px';
document.body.appendChild(historyControlDiv);

const undoButton = document.createElement('button');
undoButton.textContent = '↶ Undo';
undoButton.style.padding = '10px 20px';
undoButton.style.marginRight = '10px';
undoButton.style.fontSize = '16px';
undoButton.style.cursor = 'pointer';
historyControlDiv.appendChild(undoButton);

const redoButton = document.createElement('button');
redoButton.textContent = '↷ Redo';
redoButton.style.padding = '10px 20px';
redoButton.style.fontSize = '16px';
redoButton.style.cursor = 'pointer';
historyControlDiv.appendChild(redoButton);

const historyInfo = document.createElement('div');
historyInfo.style.marginTop = '10px';
historyInfo.style.fontFamily = 'monospace';
historyInfo.style.fontSize = '14px';
historyControlDiv.appendChild(historyInfo);

// Mostrar estado del historial
history$.subscribe(state => {
  undoButton.disabled = state.undoStack.length === 0;
  redoButton.disabled = state.redoStack.length === 0;

  historyInfo.innerHTML = `
    📚 Undo disponible: ${state.undoStack.length} veces<br>
    📚 Redo disponible: ${state.redoStack.length} veces<br>
    📝 Valor actual: ${JSON.stringify(state.current.personalInfo.firstName)}
  `;

  // Sincronizar datos del formulario
  formData$.next(state.current);
});

// Eventos de botones
undoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'UNDO' });
});

redoButton.addEventListener('click', () => {
  historyAction$.next({ type: 'REDO' });
});

// Demo: Botón de aplicación de parches
const applyPatchButton = document.createElement('button');
applyPatchButton.textContent = 'Aplicar cambio (Test Undo/Redo)';
applyPatchButton.style.padding = '10px 20px';
applyPatchButton.style.margin = '10px';
applyPatchButton.style.fontSize = '16px';
applyPatchButton.style.cursor = 'pointer';
document.body.appendChild(applyPatchButton);

applyPatchButton.addEventListener('click', () => {
  const patches: Operation[] = [
    { op: 'replace', path: '/personalInfo/firstName', value: `Taro_${Date.now()}` }
  ];
  historyAction$.next({ type: 'APPLY_PATCH', patches });
});
```

> [!TIP] Puntos clave de Undo/Redo
> - **Parche inverso** - Establecer el 4to argumento de `applyPatch` en `true` permite obtener el parche inverso
> - **Gestión de pilas** - Pila de Undo (operaciones pasadas) y pila de Redo (operaciones deshechas)
> - **Limpiar Redo con nueva operación** - Resetear pila de Redo cuando hay un nuevo cambio
> - **Patrón Reducer con `scan`** - Implementar gestión de estado similar a useReducer de React

### Patrón 3: Soporte offline (Cola de IndexedDB)

Durante el modo offline, encola los cambios en IndexedDB y sincroniza cuando se recupera la conexión.

```typescript
import { fromEvent, merge, map, filter, concatMap, catchError, of } from 'rxjs';

// Estado online/offline
const online$ = merge(
  fromEvent(window, 'online').pipe(map(() => true)),
  fromEvent(window, 'offline').pipe(map(() => false))
);

// Operaciones de IndexedDB (implementación simplificada)
class PatchQueue {
  private dbName = 'form-patches';
  private storeName = 'patches';
  private db: IDBDatabase | null = null;

  async init(): Promise<void> {
    return new Promise((resolve, reject) => {
      const request = indexedDB.open(this.dbName, 1);

      request.onerror = () => reject(request.error);
      request.onsuccess = () => {
        this.db = request.result;
        resolve();
      };

      request.onupgradeneeded = (event) => {
        const db = (event.target as IDBOpenDBRequest).result;
        if (!db.objectStoreNames.contains(this.storeName)) {
          db.createObjectStore(this.storeName, {
            keyPath: 'id',
            autoIncrement: true
          });
        }
      };
    });
  }

  async enqueue(patches: Operation[]): Promise<void> {
    if (!this.db) throw new Error('DB no inicializada');

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction([this.storeName], 'readwrite');
      const store = transaction.objectStore(this.storeName);
      const request = store.add({
        patches,
        timestamp: Date.now()
      });

      request.onsuccess = () => resolve();
      request.onerror = () => reject(request.error);
    });
  }

  async dequeueAll(): Promise<Operation[][]> {
    if (!this.db) throw new Error('DB no inicializada');

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction([this.storeName], 'readwrite');
      const store = transaction.objectStore(this.storeName);
      const request = store.getAll();

      request.onsuccess = () => {
        const items = request.result;
        // Limpiar después de obtener
        store.clear();
        resolve(items.map((item: any) => item.patches));
      };
      request.onerror = () => reject(request.error);
    });
  }
}

const patchQueue = new PatchQueue();
patchQueue.init().then(() => {
  console.log('IndexedDB inicializado');
});

// Autoguardado con soporte offline
formData$.pipe(
  pairwise(),
  map(([previous, current]) => compare(previous, current)),
  filter(patches => patches.length > 0),
  bufferTime(2000),
  filter(buffer => buffer.length > 0),
  map(buffer => buffer.flatMap(patches => patches)),
  concatMap(async (patches) => {
    const isOnline = navigator.onLine;

    if (isOnline) {
      // Online: Enviar al servidor
      try {
        return await saveToServer(patches);
      } catch (error) {
        // Fallo de envío: Agregar a cola
        await patchQueue.enqueue(patches);
        return { success: false, message: 'Agregado a cola offline' };
      }
    } else {
      // Offline: Agregar a cola
      await patchQueue.enqueue(patches);
      console.log('📴 Offline: Agregado a cola');
      return { success: false, message: 'Offline' };
    }
  })
).subscribe();

// Sincronizar al recuperar conexión
online$.pipe(
  filter(isOnline => isOnline),
  concatMap(async () => {
    console.log('🌐 Conexión recuperada: Sincronizando cola...');
    const queuedPatches = await patchQueue.dequeueAll();

    for (const patches of queuedPatches) {
      await saveToServer(patches);
    }

    return { synced: queuedPatches.length };
  })
).subscribe(result => {
  console.log(`✅ ${result.synced} parches sincronizados`);
});
```

> [!NOTE] Puntos clave del soporte offline
> - **IndexedDB** - Almacenamiento persistente del lado del navegador (mayor capacidad que LocalStorage)
> - **Encolamiento** - Acumular en cola de envío durante modo offline
> - **Detección de recuperación online** - Monitorear eventos con `window.addEventListener('online')`
> - **Garantía de orden** - Enviar parches en la cola secuencialmente con `concatMap`

---

## Sincronización en tiempo real para edición colaborativa

Implementa "edición colaborativa en tiempo real" donde múltiples usuarios editan el mismo formulario simultáneamente (funcionalidad similar a Google Docs, Notion, Figma).

### Fundamentos de Operational Transform (OT) y CRDT

En la edición colaborativa en tiempo real, la **resolución de conflictos** es el mayor desafío. Existen dos enfoques principales.

#### Operational Transform (OT)

Resuelve conflictos transformando las operaciones de cambio.

```
Usuario A: "hello" → "hello world" (agregar " world" al final)
Usuario B: "hello" → "Hi hello"   (agregar "Hi " al principio)

【Sin transformación】
Resultado: "Hi hello world" o "hello world" (último gana)

【Con OT】
- Transformar operación del Usuario A con operación del Usuario B
- Transformar operación del Usuario B con operación del Usuario A
→ Resultado: "Hi hello world" (mantener ambos cambios)
```

**Ventajas:**
- Resultado intuitivo (mantener ambos cambios)
- El servidor determina el estado final

**Desventajas:**
- Implementación compleja
- Requiere servidor

#### CRDT (Conflict-free Replicated Data Types)

Utiliza estructuras de datos que matemáticamente no generan conflictos.

```
Asignar ID único a cada carácter:

Usuario A: [h1, e2, l3, l4, o5] → [h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
Usuario B: [h1, e2, l3, l4, o5] → [H12, i13, space14, h1, e2, l3, l4, o5]

Ordenar por ID al fusionar:
→ [H12, i13, space14, h1, e2, l3, l4, o5, space6, w7, o8, r9, l10, d11]
→ "Hi hello world"
```

**Ventajas:**
- No requiere servidor (posible P2P)
- Fácil soporte offline

**Desventajas:**
- Alto uso de memoria
- Procesamiento de eliminación complejo (método Tombstone)

> [!TIP] Selección de librerías
> - **Implementación OT**: [ShareDB](https://github.com/share/sharedb) - Operational Transform
> - **Implementación CRDT**: [Yjs](https://github.com/yjs/yjs) - CRDT de alto rendimiento (recomendado)
> - **Implementación CRDT**: [Automerge](https://github.com/automerge/automerge) - Especializado en JSON

Este artículo presenta ejemplos de implementación usando **Yjs (CRDT)**.

### Patrón 4: Edición colaborativa usando Yjs

Combina Yjs y RxJS para implementar edición colaborativa en tiempo real.

```typescript
import { fromEvent, merge, Subject } from 'rxjs';
import * as Y from 'yjs';
import { WebsocketProvider } from 'y-websocket';

// Documento Yjs (estado compartido)
const ydoc = new Y.Doc();

// Datos del formulario compartido (tipo Y.Map de Yjs)
const yFormData = ydoc.getMap('formData');

// Proveedor WebSocket (conexión al servidor)
// En producción, construir servidor WebSocket propio
const wsProvider = new WebsocketProvider(
  'wss://demos.yjs.dev', // Servidor público para demostración
  'rxjs-form-demo',      // Nombre de sala
  ydoc
);

// Subject de RxJS para cambios de formulario
const formChange$ = new Subject<{
  key: string;
  value: any;
  user: string;
}>();

// Convertir cambios de Yjs a stream de RxJS
yFormData.observe(event => {
  event.changes.keys.forEach((change, key) => {
    if (change.action === 'add' || change.action === 'update') {
      formChange$.next({
        key: key,
        value: yFormData.get(key),
        user: 'remote'
      });
    }
  });
});

// UI de demostración
const collaborativeFormDiv = document.createElement('div');
collaborativeFormDiv.style.padding = '20px';
collaborativeFormDiv.style.margin = '10px';
collaborativeFormDiv.style.border = '2px solid #2196F3';
collaborativeFormDiv.style.borderRadius = '8px';
collaborativeFormDiv.style.backgroundColor = '#f5f5f5';
document.body.appendChild(collaborativeFormDiv);

const title = document.createElement('h3');
title.textContent = '🤝 Demo de edición colaborativa en tiempo real';
title.style.margin = '0 0 15px 0';
collaborativeFormDiv.appendChild(title);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.placeholder = 'Ingresa nombre (sincronizado con otros usuarios)';
nameInput.style.padding = '10px';
nameInput.style.width = '100%';
nameInput.style.fontSize = '16px';
nameInput.style.border = '2px solid #ccc';
nameInput.style.borderRadius = '4px';
nameInput.style.boxSizing = 'border-box';
collaborativeFormDiv.appendChild(nameInput);

const syncStatus = document.createElement('div');
syncStatus.style.marginTop = '10px';
syncStatus.style.fontSize = '14px';
syncStatus.style.color = '#666';
collaborativeFormDiv.appendChild(syncStatus);

const activeUsers = document.createElement('div');
activeUsers.style.marginTop = '10px';
activeUsers.style.fontSize = '14px';
activeUsers.style.color = '#666';
collaborativeFormDiv.appendChild(activeUsers);

// Monitorear estado de conexión
wsProvider.on('status', (event: { status: string }) => {
  if (event.status === 'connected') {
    syncStatus.innerHTML = '🟢 <strong>Conectado</strong> - Sincronización en tiempo real con otros usuarios';
    syncStatus.style.color = '#4CAF50';
  } else {
    syncStatus.innerHTML = '🔴 <strong>Desconectado</strong>';
    syncStatus.style.color = '#f44336';
  }
});

// Mostrar número de usuarios conectados (Awareness API)
wsProvider.awareness.on('change', () => {
  const users = Array.from(wsProvider.awareness.getStates().keys());
  activeUsers.innerHTML = `👥 Usuarios activos: ${users.length} personas`;
});

// Reflejar cambios locales en Yjs
let isRemoteChange = false;

fromEvent(nameInput, 'input').subscribe(() => {
  if (!isRemoteChange) {
    yFormData.set('name', nameInput.value);
  }
});

// Reflejar cambios remotos en UI
formChange$.subscribe(change => {
  if (change.key === 'name') {
    isRemoteChange = true;
    nameInput.value = change.value || '';
    nameInput.style.borderColor = '#4CAF50';

    setTimeout(() => {
      nameInput.style.borderColor = '#ccc';
      isRemoteChange = false;
    }, 500);
  }
});

// Configurar valor inicial
const initialName = yFormData.get('name');
if (initialName) {
  nameInput.value = initialName;
}
```

> [!NOTE] Puntos clave de Yjs
> - **Y.Doc** - Documento compartido (CRDT)
> - **Y.Map** - Tipo Map compartido (`{ key: value }`)
> - **WebsocketProvider** - Sincronización vía WebSocket
> - **Awareness API** - Información de presencia (usuarios conectados, posición del cursor)
> - **observe** - Monitorear cambios y convertir a stream de RxJS

### Patrón 5: Gestión de presencia (Compartir posición del cursor)

Visualiza quién está editando dónde.

```typescript
import { throttleTime } from 'rxjs';

interface UserPresence {
  userId: string;
  name: string;
  color: string;
  cursorPosition: number;
  timestamp: number;
}

// Generar color aleatorio
function generateRandomColor(): string {
  const colors = ['#f44336', '#E91E63', '#9C27B0', '#673AB7', '#3F51B5', '#2196F3', '#00BCD4', '#009688'];
  return colors[Math.floor(Math.random() * colors.length)];
}

// ID de usuario propio (generado aleatoriamente)
const myUserId = `user_${Math.random().toString(36).substr(2, 9)}`;
const myColor = generateRandomColor();

// Configurar Awareness (información de presencia)
wsProvider.awareness.setLocalState({
  userId: myUserId,
  name: `Usuario${myUserId.slice(-4)}`,
  color: myColor
});

// Detectar cambio de posición del cursor (limitar frecuencia de envío con throttle)
fromEvent(nameInput, 'selectionchange').pipe(
  throttleTime(200)
).subscribe(() => {
  const cursorPosition = nameInput.selectionStart || 0;

  wsProvider.awareness.setLocalStateField('cursorPosition', cursorPosition);
});

// Mostrar posición del cursor de otros usuarios
const cursorOverlay = document.createElement('div');
cursorOverlay.style.position = 'relative';
cursorOverlay.style.marginTop = '10px';
cursorOverlay.style.fontSize = '12px';
collaborativeFormDiv.appendChild(cursorOverlay);

wsProvider.awareness.on('change', () => {
  const states = wsProvider.awareness.getStates();
  const cursors: string[] = [];

  states.forEach((state: any, clientId: number) => {
    if (state.userId !== myUserId) {
      cursors.push(
        `<span style="color: ${state.color}">● ${state.name}</span> (posición: ${state.cursorPosition || 0})`
      );
    }
  });

  cursorOverlay.innerHTML = cursors.length > 0
    ? `📍 Otros usuarios: ${cursors.join(', ')}`
    : '📍 No hay otros usuarios';
});
```

> [!TIP] Puntos clave de gestión de presencia
> - **`awareness.setLocalState`** - Compartir información propia
> - **`awareness.getStates`** - Obtener información de todos los usuarios
> - **`throttleTime`** - Limitar frecuencia de envío de movimiento del cursor (200ms)
> - **Codificación por colores** - Asignar color a cada usuario para mejorar visibilidad

### Patrón 6: Manejo de errores y reconexión

Implementa reconexión y manejo de errores cuando se desconecta WebSocket.

```typescript
import { timer, takeUntil, Subject } from 'rxjs';

const disconnect$ = new Subject<void>();

// Detectar desconexión de WebSocket
wsProvider.on('connection-close', () => {
  console.warn('⚠️ WebSocket desconectado');
  syncStatus.innerHTML = '🟡 <strong>Reconectando...</strong>';
  syncStatus.style.color = '#FF9800';

  // Intentar reconexión después de 5 segundos
  timer(5000).pipe(
    takeUntil(disconnect$)
  ).subscribe(() => {
    console.log('🔄 Intentando reconexión');
    wsProvider.connect();
  });
});

// Manejo de errores de WebSocket
wsProvider.on('connection-error', (error: Error) => {
  console.error('❌ Error de WebSocket:', error);
  syncStatus.innerHTML = `❌ <strong>Error:</strong> ${error.message}`;
  syncStatus.style.color = '#f44336';
});

// Limpieza
window.addEventListener('beforeunload', () => {
  disconnect$.next();
  wsProvider.disconnect();
  ydoc.destroy();
});
```

> [!WARNING] Precauciones para entorno de producción
> - **Servidor WebSocket propio** - `wss://demos.yjs.dev` es para desarrollo. En producción, construir [y-websocket-server](https://github.com/yjs/y-websocket)
> - **Autenticación** - Implementar autenticación por token al conectar WebSocket
> - **Escalado** - Compartir estado entre servidores WebSocket usando Redis, etc.
> - **Persistencia** - Guardar documentos Yjs en base de datos (`y-leveldb`, `y-indexeddb`)

---

## Resumen

Este artículo explicó patrones avanzados de formularios usando JSON Patch.

### Puntos importantes

> [!IMPORTANT] Criterios de selección de patrones de JSON Patch
>
> **Cuando el procesamiento normal de formularios es suficiente:**
> - Número de campos: ~20
> - Autoguardado: No necesario o OK con envío completo
> - Undo/Redo: No necesario
> - Edición colaborativa: No necesaria
> → Usar [Patrones normales de manejo de formularios](./form-handling.md)
>
> **Cuando JSON Patch es necesario:**
> - Número de campos: Más de 100
> - Autoguardado: Esencial (enviar solo diferencias)
> - Undo/Redo: Esencial
> - Edición colaborativa: Sincronización en tiempo real necesaria
> → Usar los patrones de este artículo

### Resumen de patrones de implementación

| Patrón | Caso de uso | Tecnologías principales |
|---------|------------|---------|
| **Autoguardado básico** | Envío de diferencias en formularios a gran escala | `pairwise` + `bufferTime` + `concatMap` |
| **Undo/Redo** | Gestión de historial de operaciones | Parches inversos + `scan` |
| **Soporte offline** | Respuesta ante desconexión de red | IndexedDB + Encolamiento |
| **Edición colaborativa (Yjs)** | Sincronización en tiempo real | Yjs (CRDT) + WebSocket |
| **Gestión de presencia** | Compartir posición del cursor | Awareness API + `throttleTime` |

### Próximos pasos

- **[Procesamiento de datos en tiempo real](./real-time-data.md)** - Implementación detallada de WebSocket
- **[Prácticas de manejo de errores](./error-handling-patterns.md)** - Manejo de errores en comunicación de API
- **[Estrategias de caché](./caching-strategies.md)** - Gestión de caché de datos

## Recursos de referencia

### Especificaciones estándar

- [RFC 6902: JSON Patch](https://datatracker.ietf.org/doc/html/rfc6902) - Especificación de JSON Patch
- [RFC 6901: JSON Pointer](https://datatracker.ietf.org/doc/html/rfc6901) - Especificación de JSON Pointer

### Librerías

- [fast-json-patch](https://github.com/Starcounter-Jack/JSON-Patch) - Implementación de JSON Patch (conforme a RFC)
- [Yjs](https://docs.yjs.dev/) - Implementación CRDT (edición colaborativa)
- [ShareDB](https://share.github.io/sharedb/) - Implementación de Operational Transform
- [Automerge](https://automerge.org/) - CRDT especializado en JSON

### Recursos de aprendizaje

- [CRDTs: The Hard Parts](https://www.youtube.com/watch?v=x7drE24geUw) - Comprensión profunda de CRDT (video)
- [Operational Transformation Explained](https://operational-transformation.github.io/) - Explicación detallada de OT
- [Real-time Collaborative Editing](https://pierrehedkvist.com/posts/1-creating-a-collaborative-editor) - Guía de implementación de edición colaborativa
