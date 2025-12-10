---
description: "El operador expand es un operador de RxJS que genera nuevos Observables de cada valor y expande los resultados recursivamente. Implementa de manera segura en TypeScript patrones de obtención de datos recursivos como el recorrido de estructuras de árbol, paginación de API, scroll infinito y resolución de dependencias."
---

# expand - Expansión recursiva

El operador `expand` realiza una transformación recursiva que **genera un nuevo Observable de cada valor y expande esos resultados de la misma manera**. Es ideal para procesos que expanden valores sucesivamente, como el recorrido de estructuras de árbol, paginación de API y cálculos recursivos.


## 🔰 Sintaxis básica y uso

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// Proceso recursivo que duplica
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Prevenir bucle infinito
).subscribe(console.log);
// Salida: 1, 2, 4, 8, 16
```

**Flujo de operación**:
1. Se emite el valor inicial `1`
2. La función `expand` recibe `1` y devuelve `of(2)`
3. Se emite `2` y la función `expand` se llama nuevamente
4. La función `expand` recibe `2` y devuelve `of(4)`
5. Esta repetición...

> [!WARNING]
> `expand` causará un **bucle infinito** si no se especifica una condición de finalización. Asegúrate de establecer una condición de finalización, como `take` o devolver `EMPTY` condicionalmente.

[🌐 Documentación oficial de RxJS - `expand`](https://rxjs.dev/api/operators/expand)


## 🔄 Diferencia con mergeMap

`expand` es similar a `mergeMap`, pero difiere en que **procesa recursivamente también los resultados de los Observables generados**.

```ts
import { of } from 'rxjs';
import { mergeMap, expand, take } from 'rxjs';

const double = (x: number) => of(x * 2);

// mergeMap: transformación solo una vez
of(1).pipe(
  mergeMap(double),
  take(5)
).subscribe(console.log);
// Salida: 2
// (solo un valor, 2 no se transforma nuevamente)

// expand: transformación recursiva
of(1).pipe(
  expand(double),
  take(5)
).subscribe(console.log);
// Salida: 1, 2, 4, 8, 16
// (cada resultado se transforma nuevamente)
```

| Operador | Procesamiento | Recursivo | Caso de uso |
|---|---|---|---|
| `mergeMap` | Transforma cada valor solo una vez | ❌ | Transformación asíncrona normal |
| `expand` | Transforma resultados recursivamente | ✅ | Recorrido de árbol, paginación, cálculo recursivo |


## 💡 Patrones de uso típicos

### 1. Procesamiento recursivo con condición de finalización

```ts
import { of, EMPTY } from 'rxjs';
import { expand } from 'rxjs';

// Duplicar hasta menos de 10
of(1).pipe(
  expand(x => {
    const next = x * 2;
    return next < 10 ? of(next) : EMPTY;
  })
).subscribe(console.log);
// Salida: 1, 2, 4, 8
// (16 es >= 10, por lo que devuelve EMPTY y termina)
```

### 2. Recorrido de estructura de árbol

```ts
import { of, from, EMPTY } from 'rxjs';
import { expand, mergeMap } from 'rxjs';

interface TreeNode {
  id: number;
  name: string;
  children?: TreeNode[];
}

const tree: TreeNode = {
  id: 1,
  name: 'Root',
  children: [
    {
      id: 2,
      name: 'Child 1',
      children: [
        { id: 4, name: 'Grandchild 1' },
        { id: 5, name: 'Grandchild 2' }
      ]
    },
    {
      id: 3,
      name: 'Child 2',
      children: [
        { id: 6, name: 'Grandchild 3' }
      ]
    }
  ]
};

// Recorrer todo el árbol
of(tree).pipe(
  expand(node =>
    node.children && node.children.length > 0
      ? from(node.children)
      : EMPTY
  )
).subscribe(node => {
  console.log(`ID: ${node.id}, Name: ${node.name}`);
});
// Salida:
// ID: 1, Name: Root
// ID: 2, Name: Child 1
// ID: 3, Name: Child 2
// ID: 4, Name: Grandchild 1
// ID: 5, Name: Grandchild 2
// ID: 6, Name: Grandchild 3
```

### 3. Paginación de API

```ts
import { of, EMPTY } from 'rxjs';
import { expand, mergeMap } from 'rxjs';

interface PageResponse {
  data: string[];
  nextPage: number | null;
}

function fetchPage(page: number): Promise<PageResponse> {
  // Simular solicitud API
  return new Promise(resolve => {
    setTimeout(() => {
      if (page > 3) {
        resolve({ data: [], nextPage: null });
      } else {
        resolve({
          data: [`Item ${page}-1`, `Item ${page}-2`, `Item ${page}-3`],
          nextPage: page + 1
        });
      }
    }, 100);
  });
}

// Obtener todas las páginas secuencialmente
of(1).pipe(
  expand(page => {
    return page > 0 ? of(page) : EMPTY;
  }),
  mergeMap(page => fetchPage(page)),
  expand(response =>
    response.nextPage
      ? of(response.nextPage).pipe(
          mergeMap(nextPage => fetchPage(nextPage))
        )
      : EMPTY
  )
).subscribe(response => {
  console.log(`Datos de página:`, response.data);
});
```

#### Implementación de paginación más práctica

```ts
import { defer, EMPTY, lastValueFrom } from 'rxjs';
import { expand, map, reduce, tap } from 'rxjs';

interface PaginatedResponse<T> {
  items: T[];
  nextCursor: string | null;
}

function fetchPagedData<T>(
  fetchFn: (cursor: string | null) => Promise<PaginatedResponse<T>>
): Promise<T[]> {
  return lastValueFrom(
    defer(() => fetchFn(null)).pipe(
      expand(response =>
        response.nextCursor
          ? defer(() => fetchFn(response.nextCursor))
          : EMPTY
      ),
      map(response => response.items),
      reduce((acc, items) => [...acc, ...items], [] as T[])
    )
  );
}

// Crear elementos UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Ejemplo de implementación de paginación';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Obtener todos los datos';
container.appendChild(button);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#f0f0f0';
container.appendChild(status);

const output = document.createElement('pre');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.backgroundColor = '#f9f9f9';
output.style.maxHeight = '300px';
output.style.overflow = 'auto';
container.appendChild(output);

// Ejemplo de uso: obtener datos de usuario con API simulada
interface User {
  id: number;
  name: string;
  email: string;
}

// Simular API simulada
async function fetchUsers(cursor: string | null): Promise<PaginatedResponse<User>> {
  // Simular solicitud API (retardo de 100ms)
  await new Promise(resolve => setTimeout(resolve, 100));

  const page = cursor ? parseInt(cursor) : 1;
  const pageSize = 5;
  const totalPages = 4;

  if (page > totalPages) {
    return { items: [], nextCursor: null };
  }

  const items: User[] = Array.from({ length: pageSize }, (_, i) => ({
    id: (page - 1) * pageSize + i + 1,
    name: `User ${(page - 1) * pageSize + i + 1}`,
    email: `user${(page - 1) * pageSize + i + 1}@example.com`
  }));

  return {
    items,
    nextCursor: page < totalPages ? String(page + 1) : null
  };
}

// Obtener todos los datos al hacer clic en el botón
button.addEventListener('click', async () => {
  button.disabled = true;
  status.textContent = 'Obteniendo datos...';
  output.textContent = '';

  try {
    const allUsers = await fetchPagedData(fetchUsers);

    status.textContent = `Obtención completa: ${allUsers.length} datos de usuario`;
    output.textContent = JSON.stringify(allUsers, null, 2);

    console.log(`Total de usuarios: ${allUsers.length}`);
    console.log('Datos de usuario:', allUsers);
  } catch (error) {
    status.textContent = `Error: ${error}`;
  } finally {
    button.disabled = false;
  }
});
```


## 🧠 Ejemplo de código práctico (visualización de jerarquía de directorios)

Ejemplo de recorrido recursivo de la estructura de directorios de un sistema de archivos.

```ts
import { of, from, EMPTY } from 'rxjs';
import { expand, tap } from 'rxjs';

interface FileSystemItem {
  name: string;
  type: 'file' | 'directory';
  path: string;
  children?: FileSystemItem[];
  level: number;
}

// Estructura de sistema de archivos de muestra
const fileSystem: FileSystemItem = {
  name: 'root',
  type: 'directory',
  path: '/root',
  level: 0,
  children: [
    {
      name: 'src',
      type: 'directory',
      path: '/root/src',
      level: 1,
      children: [
        { name: 'index.ts', type: 'file', path: '/root/src/index.ts', level: 2 },
        { name: 'utils.ts', type: 'file', path: '/root/src/utils.ts', level: 2 },
        {
          name: 'components',
          type: 'directory',
          path: '/root/src/components',
          level: 2,
          children: [
            { name: 'Button.tsx', type: 'file', path: '/root/src/components/Button.tsx', level: 3 },
            { name: 'Input.tsx', type: 'file', path: '/root/src/components/Input.tsx', level: 3 }
          ]
        }
      ]
    },
    {
      name: 'docs',
      type: 'directory',
      path: '/root/docs',
      level: 1,
      children: [
        { name: 'README.md', type: 'file', path: '/root/docs/README.md', level: 2 }
      ]
    },
    { name: 'package.json', type: 'file', path: '/root/package.json', level: 1 }
  ]
};

// Crear elementos UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Visualización de jerarquía de directorios';
container.appendChild(title);

const output = document.createElement('pre');
output.style.padding = '10px';
output.style.backgroundColor = '#f5f5f5';
output.style.fontFamily = 'monospace';
output.style.fontSize = '14px';
container.appendChild(output);

const stats = document.createElement('div');
stats.style.marginTop = '10px';
stats.style.padding = '10px';
stats.style.backgroundColor = '#e3f2fd';
container.appendChild(stats);

let fileCount = 0;
let dirCount = 0;

// Expandir recursivamente la estructura de directorios
of(fileSystem).pipe(
  expand(item => {
    if (item.type === 'directory' && item.children && item.children.length > 0) {
      return from(
        item.children.map(child => ({
          ...child,
          level: item.level + 1
        }))
      );
    }
    return EMPTY;
  }),
  tap(item => {
    if (item.type === 'file') {
      fileCount++;
    } else {
      dirCount++;
    }
  })
).subscribe({
  next: item => {
    const indent = '  '.repeat(item.level);
    const icon = item.type === 'directory' ? '📁' : '📄';
    output.textContent += `${indent}${icon} ${item.name}\n`;
  },
  complete: () => {
    stats.textContent = `Directorios: ${dirCount}, Archivos: ${fileCount}`;
  }
});
```


## 📋 Uso con seguridad de tipos

Ejemplo de implementación con seguridad de tipos utilizando genéricos en TypeScript.

```ts
import { Observable, of, from, EMPTY } from 'rxjs';
import { expand, filter, take, defaultIfEmpty, reduce } from 'rxjs';

interface Node<T> {
  value: T;
  children?: Node<T>[];
}

class TreeTraversal<T> {
  /**
   * Recorrer estructura de árbol con búsqueda en anchura
   */
  traverseBFS(root: Node<T>): Observable<Node<T>> {
    return of(root).pipe(
      expand(node =>
        node.children && node.children.length > 0
          ? from(node.children)
          : EMPTY
      )
    );
  }

  /**
   * Buscar el primer nodo que coincida con la condición
   */
  findNode(
    root: Node<T>,
    predicate: (value: T) => boolean
  ): Observable<Node<T> | undefined> {
    return this.traverseBFS(root).pipe(
      filter(node => predicate(node.value)),
      take(1),
      defaultIfEmpty(undefined as Node<T> | undefined)
    );
  }

  /**
   * Contar todos los nodos del árbol
   */
  countNodes(root: Node<T>): Observable<number> {
    return this.traverseBFS(root).pipe(
      reduce((count) => count + 1, 0)
    );
  }

  /**
   * Obtener todos los nodos con un valor específico
   */
  findAllNodes(
    root: Node<T>,
    predicate: (value: T) => boolean
  ): Observable<Node<T>[]> {
    return this.traverseBFS(root).pipe(
      filter(node => predicate(node.value)),
      reduce((acc, node) => [...acc, node], [] as Node<T>[])
    );
  }
}

// Ejemplo de uso
const tree: Node<string> = {
  value: 'A',
  children: [
    {
      value: 'B',
      children: [
        { value: 'D' },
        { value: 'E' }
      ]
    },
    {
      value: 'C',
      children: [
        { value: 'F' }
      ]
    }
  ]
};

const traversal = new TreeTraversal<string>();

// Recorrer todo el árbol
traversal.traverseBFS(tree).subscribe(node => {
  console.log(`Visitar: ${node.value}`);
});
// Salida: Visitar: A, Visitar: B, Visitar: C, Visitar: D, Visitar: E, Visitar: F

// Buscar un nodo específico
traversal.findNode(tree, value => value === 'D').subscribe(node => {
  console.log(`Nodo encontrado: ${node?.value}`);
});
// Salida: Nodo encontrado: D

// Contar nodos
traversal.countNodes(tree).subscribe(count => {
  console.log(`Nodos en el árbol: ${count}`);
});
// Salida: Nodos en el árbol: 6

// Obtener todos los nodos que coincidan con la condición
traversal.findAllNodes(tree, value => value.length === 1).subscribe(nodes => {
  console.log(`Nodos de un solo carácter: ${nodes.map(n => n.value).join(', ')}`);
});
// Salida: Nodos de un solo carácter: A, B, C, D, E, F
```


## 🎯 Combinación con Scheduler

`expand` opera de forma síncrona por defecto, pero se puede controlar de forma asíncrona usando schedulers.

```ts
import { of, asyncScheduler } from 'rxjs';
import { expand, take } from 'rxjs';

// Síncrono (predeterminado)
console.log('Inicio de expand síncrono');
of(1).pipe(
  expand(x => of(x * 2)),
  take(5)
).subscribe(x => console.log('Síncrono:', x));
console.log('Fin de expand síncrono');
// Salida:
// Inicio de expand síncrono
// Síncrono: 1
// Síncrono: 2
// Síncrono: 4
// Síncrono: 8
// Síncrono: 16
// Fin de expand síncrono

// Asíncrono (usando asyncScheduler)
console.log('Inicio de expand asíncrono');
of(1, asyncScheduler).pipe(
  expand(x => of(x * 2, asyncScheduler)),
  take(5)
).subscribe(x => console.log('Asíncrono:', x));
console.log('Fin de expand asíncrono');
// Salida:
// Inicio de expand asíncrono
// Fin de expand asíncrono
// Asíncrono: 1
// Asíncrono: 2
// Asíncrono: 4
// Asíncrono: 8
// Asíncrono: 16
```

> [!TIP]
> Al procesar grandes cantidades de datos, usar `asyncScheduler` puede mantener la capacidad de respuesta de la UI sin bloquear el hilo principal. Para más detalles, consulta [Tipos de Scheduler y uso](/es/guide/schedulers/types).


## 🔄 Ejemplo de cálculo recursivo

### Secuencia de Fibonacci

```ts
import { of, EMPTY } from 'rxjs';
import { expand, map, take } from 'rxjs';

interface FibState {
  current: number;
  next: number;
}

// Generar secuencia de Fibonacci
of({ current: 0, next: 1 } as FibState).pipe(
  expand(state =>
    state.current < 100
      ? of({ current: state.next, next: state.current + state.next })
      : EMPTY
  ),
  map(state => state.current),
  take(10)
).subscribe(n => console.log(n));
// Salida: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### Cálculo de factorial

```ts
import { of, EMPTY, Observable } from 'rxjs';
import { expand, reduce } from 'rxjs';

interface FactorialState {
  n: number;
  result: number;
}

function factorial(n: number): Observable<number> {
  return of({ n, result: 1 } as FactorialState).pipe(
    expand(state =>
      state.n > 1
        ? of({ n: state.n - 1, result: state.result * state.n })
        : EMPTY
    ),
    reduce((acc, state) => state.result, 1)
  );
}

factorial(5).subscribe(result => {
  console.log('5! =', result); // 5! = 120
});
```


## ⚠️ Errores comunes

> [!WARNING]
> El error más común con `expand` es **olvidarse de establecer una condición de finalización, causando un bucle infinito**.

### Incorrecto: sin condición de finalización

```ts
import { of } from 'rxjs';
import { expand } from 'rxjs';

// ❌ Mal ejemplo: bucle infinito
of(1).pipe(
  expand(x => of(x + 1))
).subscribe(console.log);
// Causa fuga de memoria y congelación del navegador
```

### Correcto: con condición de finalización

```ts
import { of, EMPTY } from 'rxjs';
import { expand, take, takeWhile } from 'rxjs';

// ✅ Buen ejemplo 1: limitar cantidad con take
of(1).pipe(
  expand(x => of(x + 1)),
  take(10)
).subscribe(console.log);

// ✅ Buen ejemplo 2: devolver EMPTY condicionalmente
of(1).pipe(
  expand(x => x < 10 ? of(x + 1) : EMPTY)
).subscribe(console.log);

// ✅ Buen ejemplo 3: limitar con condición usando takeWhile
of(1).pipe(
  expand(x => of(x + 1)),
  takeWhile(x => x <= 10)
).subscribe(console.log);
```

> [!IMPORTANT]
> En el procesamiento recursivo, siempre aclara la condición de finalización y prevén bucles infinitos usando `take`, `takeWhile` o devolviendo `EMPTY` según la condición.


## 🎓 Resumen

### Cuándo usar expand
- ✅ Al recorrer recursivamente estructuras de árbol o grafos
- ✅ Al obtener todos los datos con paginación de API
- ✅ Al realizar cálculos recursivos (Fibonacci, factorial, etc.)
- ✅ Al recorrer estructura de directorios o sistema de archivos
- ✅ Al explorar organigramas o datos jerárquicos

### Cuándo usar mergeMap
- ✅ Cuando es suficiente transformar cada valor solo una vez
- ✅ Transformación asíncrona normal que no requiere procesamiento recursivo

### Puntos de atención
- ⚠️ **Establecer siempre condición de finalización** (prevenir bucle infinito)
- ⚠️ Cuidado con el consumo de memoria (al expandir grandes cantidades de datos)
- ⚠️ Opera de forma síncrona, considerar usar `asyncScheduler` para grandes cantidades de datos
- ⚠️ La depuración es difícil, por lo que es bueno registrar el estado intermedio con `tap`


## 🚀 Próximos pasos

- **[mergeMap](./mergeMap)** - Aprender transformación asíncrona normal
- **[switchMap](./switchMap)** - Aprender transformación que cambia al procesamiento más reciente
- **[concatMap](./concatMap)** - Aprender transformación de ejecución secuencial
- **[Tipos de Scheduler y uso](/es/guide/schedulers/types)** - Aprender combinación de expand y scheduler
- **[Ejemplos prácticos de operadores de transformación](./practical-use-cases)** - Aprender casos de uso reales
