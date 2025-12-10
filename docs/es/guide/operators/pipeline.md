---
description: "Aprende la construcción de pipelines RxJS con el método pipe: Transforma, filtra y combina flujos de datos declarativamente con ejemplos completos de encadenamiento de operadores"
---

# ¿Qué es el Pipeline de RxJS?

El pipelining en RxJS es un mecanismo para aplicar una serie de operaciones (operadores) a un Observable en secuencia. El pipelining te permite transformar, filtrar y combinar flujos de datos en múltiples etapas, permitiéndote controlar el flujo de datos en un estilo de programación declarativo.

## Estructura Básica de un Pipeline

[📘 RxJS Official: pipe()](https://rxjs.dev/api/index/function/pipe)

El método `pipe()` de RxJS se usa para construir un pipeline. La sintaxis es la siguiente.

```ts
import { Observable } from 'rxjs';
import { map, filter, tap } from 'rxjs';

const source$: Observable<number> = // Algún Observable
source$.pipe(
  // Encadena múltiples operadores
  operator1(),
  operator2(),
  operator3(),
  // ...
).subscribe(value => {
  // Procesa el resultado
});
```

## Ejemplos Prácticos

### Conversión Básica de Datos

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// Flujo de números
const numbers$ = of(1, 2, 3, 4, 5);

// Construye un pipeline
numbers$.pipe(
  // Pasa solo números pares
  filter(n => n % 2 === 0),
  // Duplica el valor
  map(n => n * 2)
).subscribe(
  value => console.log(`Resultado: ${value}`)
);

// Salida:
// Resultado: 4
// Resultado: 8
```

### Procesamiento Complejo de Datos

```ts
import { fromEvent, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

type User = {
  id: number;
  name: string;
  username: string;
  email: string;
};
type Post = {
  userId: number;
  id: number;
  title: string;
  body: string;
};

// Crear elementos DOM
const searchButton = document.createElement('button');
searchButton.innerText = 'Buscar';
document.body.appendChild(searchButton);

const resultBox = document.createElement('div');
resultBox.id = 'results';
document.body.appendChild(resultBox);

// Solicitud API al hacer clic en el botón
fromEvent(searchButton, 'click')
  .pipe(
    switchMap(() =>
      // Primera llamada API
      ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
        // Segunda llamada API para obtener publicaciones del usuario
        switchMap((user) => {
          const header = document.createElement('h3');
          header.textContent = `Usuario: ${user.name}`;
          resultBox.innerHTML = ''; // Limpiar resultados anteriores
          resultBox.appendChild(header);

          return ajax.getJSON<Post[]>(
            `https://jsonplaceholder.typicode.com/posts?userId=${user.id}`
          );
        }),
        // Obtener solo las primeras 3 publicaciones
        map((posts) => posts.slice(0, 3))
      )
    )
  )
  .subscribe((posts) => {
    // Mostrar publicaciones en la pantalla
    resultBox.innerHTML += '<h4>Publicaciones:</h4>';
    posts.forEach((post) => {
      const div = document.createElement('div');
      div.innerHTML = `<strong>${post.title}</strong><p>${post.body}</p>`;
      resultBox.appendChild(div);
    });
  });

```


## Ventajas del Pipeline

Primero, veamos el código escrito de manera imperativa. Como se muestra a continuación, el pipelining de RxJS te permite reescribirlo en una forma más legible y mantenible, haciendo clara la intención del proceso.

### 1. Mejor Legibilidad y Mantenibilidad

```ts
// Procesamiento en estilo imperativo
const data = [
  { id: 3, active: true },
  { id: 1, active: false },
  { id: 2, active: true }
];

const activeItems = [];
for (const item of data) {
  if (item.active) {
    activeItems.push({ ...item, label: `Item #${item.id}` });
  }
}
activeItems.sort((a, b) => a.id - b.id);

const div1 = document.createElement('div');
div1.innerHTML = '<h3>Estilo Imperativo</h3>';
activeItems.forEach(item => {
  const p = document.createElement('p');
  p.textContent = item.label;
  div1.appendChild(p);
});
document.body.appendChild(div1);
```
⬇️⬇️⬇️
```ts
import { of } from 'rxjs';
import { filter, map, toArray } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>Mejor Legibilidad y Mantenibilidad</h3>';
document.body.appendChild(output);

of(
  { id: 3, active: true },
  { id: 1, active: false },
  { id: 2, active: true }
).pipe(
  filter(item => item.active),
  map(item => ({ ...item, label: `Item #${item.id}` })),
  toArray(),
  map(array => array.sort((a, b) => a.id - b.id))
).subscribe(sorted => {
  sorted.forEach(item => {
    const div = document.createElement('div');
    div.textContent = item.label;
    output.appendChild(div);
  });
});
```

El pipelining hace que el flujo de datos sea claro y elimina la necesidad de reasignar variables o gestionar estados intermedios.



El código procedimental como el anterior se puede escribir de manera concisa en un estilo declarativo usando el pipelining de RxJS. A continuación se muestra un ejemplo.

### 2. Estilo de Programación Declarativo

El pipelining promueve un estilo declarativo que establece explícitamente "qué hacer". Esto hace que la intención del código sea más clara.

```ts
// Procesamiento en estilo procedimental
const usersList = [
  { status: 'active', firstName: 'Taro', lastName: 'Yamada', email: 'taro@example.com' },
  { status: 'inactive', firstName: 'Hanako', lastName: 'Yamada', email: 'hanako@example.com' },
  { status: 'active', firstName: 'John', lastName: 'Doe', email: 'john@example.com' }
];

const activeUsers2 = [];
for (const user of usersList) {
  if (user.status === 'active') {
    const name = `${user.firstName} ${user.lastName}`;
    activeUsers2.push({ name, email: user.email });
  }
}

const div2 = document.createElement('div');
div2.innerHTML = '<h3>Estilo Procedimental</h3>';
activeUsers2.forEach(user => {
  const p = document.createElement('p');
  p.textContent = `${user.name} (${user.email})`;
  div2.appendChild(p);
});
document.body.appendChild(div2);
```

⬇️⬇️⬇️

```ts
// Estilo de programación declarativo
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

const out2 = document.createElement('div');
out2.innerHTML = '<h3>Estilo Declarativo</h3>';
document.body.appendChild(out2);

const users = [
  { status: 'active', firstName: 'Taro', lastName: 'Yamada', email: 'taro@example.com' },
  { status: 'inactive', firstName: 'Hanako', lastName: 'Yamada', email: 'hanako@example.com' },
  { status: 'active', firstName: 'John', lastName: 'Doe', email: 'john@example.com' }
];

from(users).pipe(
  filter(user => user.status === 'active'),
  map(user => ({
    name: `${user.firstName} ${user.lastName}`,
    email: user.email
  }))
).subscribe(user => {
  const div = document.createElement('div');
  div.textContent = `${user.name} (${user.email})`;
  out2.appendChild(div);
});
```


De manera similar aquí, tomemos código que describe procesamiento de manera procedimental y reorganicémoslo con pipelining. El procesamiento complejo se puede construir simplemente componiendo operadores individuales.

### 3. Composabilidad

El pipelining te permite construir procesamiento complejo combinando operaciones pequeñas.

```ts
// Procesamiento de estilo procedimental (imperativo)
const rawUsers = [
  { firstName: 'Alice', lastName: 'Smith', status: 'active' },
  { firstName: 'Bob', lastName: 'Brown', status: 'inactive' },
  { firstName: 'Carol', lastName: 'Jones', status: 'active' }
];

const activeUsers = [];
for (const user of rawUsers) {
  if (user.status === 'active') {
    const fullName = `${user.firstName} ${user.lastName}`;
    activeUsers.push({ ...user, fullName });
  }
}
activeUsers.sort((a, b) => a.fullName.localeCompare(b.fullName));

const div0 = document.createElement('div');
div0.innerHTML = '<h3>Estilo Procedimental</h3>';
activeUsers.forEach(user => {
  const p = document.createElement('p');
  p.textContent = user.fullName;
  div0.appendChild(p);
});
document.body.appendChild(div0);
```

⬇️⬇️⬇️

```ts
// Estilo de programación declarativo
import { from } from 'rxjs';
import { filter, map, toArray } from 'rxjs';

const out3 = document.createElement('div');
out3.innerHTML = '<h3>Composabilidad</h3>';
document.body.appendChild(out3);

const users3 = [
  { firstName: 'Alice', lastName: 'Smith', status: 'active' },
  { firstName: 'Bob', lastName: 'Brown', status: 'inactive' },
  { firstName: 'Carol', lastName: 'Jones', status: 'active' }
];

const filterActive = filter((user: any) => user.status === 'active');
const formatFullName = map((user: any) => ({ ...user, fullName: `${user.firstName} ${user.lastName}` }));
const collectAndSort = toArray();
const sortByName = map((users: any[]) => users.sort((a, b) => a.fullName.localeCompare(b.fullName)));

from(users3).pipe(
  filterActive,
  formatFullName,
  collectAndSort,
  sortByName
).subscribe(users => {
  users.forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.fullName;
    out3.appendChild(div);
  });
});
```

## Técnicas de Optimización de Pipeline

### 1. Importancia del Orden de Operadores

El orden de los operadores tiene un impacto significativo tanto en el rendimiento como en la funcionalidad.

```ts
// Ineficiente: map se aplica a todos los elementos
observable$.pipe(
  map(x => expensiveTransformation(x)),
  filter(x => x > 10)
)

// Eficiente: filter se ejecuta primero, reduciendo elementos a transformar
observable$.pipe(
  filter(x => x > 10),
  map(x => expensiveTransformation(x))
)
```

### 2. Creación de Pipelines Personalizados

El procesamiento complejo se puede extraer en pipelines reutilizables.

```ts
import { Observable, pipe } from 'rxjs';
import { filter, map } from 'rxjs';

// Función de pipeline personalizada
export function filterAndTransform<T, R>(
  filterFn: (value: T) => boolean,
  transformFn: (value: T) => R
) {
  return pipe(
    filter(filterFn),
    map(transformFn)
  );
}

// Ejemplo de uso
observable$.pipe(
  filterAndTransform(
    x => x > 10,
    x => x * 2
  )
).subscribe(console.log);
```

## Errores Comunes con Pipelines

### 1. Orden Incorrecto de Operadores

```ts
// ❌ Si aplicas filter antes de debounceTime,
// filter se ejecutará para cada entrada, reduciendo el efecto del debounce
inputEvents$.pipe(
  filter(text => text.length > 2),
  debounceTime(300)
)

// ✅ Aplica debounceTime primero
inputEvents$.pipe(
  debounceTime(300),
  filter(text => text.length > 2)
)
```

### 2. Efectos Secundarios en el Pipeline

```ts
// ❌ Ejecutar efectos secundarios directamente en el pipeline
observable$.pipe(
  map(data => {
    // Efectos secundarios (mal ejemplo)
    console.log(data);
    localStorage.setItem('lastData', JSON.stringify(data));
    return data;
  })
)

// ✅ Usa el operador tap
observable$.pipe(
  tap(data => {
    console.log(data);
    localStorage.setItem('lastData', JSON.stringify(data));
  }),
  // Realiza transformación de datos con map
  map(data => transformData(data))
)
```

## Resumen

Los pipelines de RxJS son un mecanismo poderoso para gestionar flujos de datos asíncronos complejos de manera declarativa y componible. Los pipelines correctamente diseñados pueden mejorar enormemente la legibilidad, mantenibilidad y reusabilidad del código.

Al diseñar pipelines, es buena idea tener en cuenta los siguientes puntos:

1. Elegir la secuencia más eficiente de operadores
2. Extraer y reutilizar patrones de pipeline comunes
3. Aislar efectos secundarios con operadores `tap`
4. Asegurar que cada paso en el pipeline tenga una sola responsabilidad

Este enfoque orientado a pipelines es especialmente poderoso en escenarios como procesamiento complejo de eventos de UI, solicitudes API y gestión de estado.
