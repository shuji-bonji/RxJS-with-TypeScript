---
description: La Función de Creación forkJoin emite el último valor de cada uno como un array u objeto después de que todos los múltiples Observables hayan completado. Esto es ideal cuando se ejecutan múltiples solicitudes API en paralelo y todos los resultados están disponibles antes de procesar.
---

# forkJoin - emitir todos los últimos valores juntos

`forkJoin` es una Función de Creación que emite el último valor de cada Observable como un array u objeto después de que **todos** los Observables hayan completado.
Esto es muy útil cuando quieres usar todos los Observables a la vez.


## Sintaxis básica y uso

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

const user$ = of('Usuario A').pipe(delay(1000));
const posts$ = of('Lista de publicaciones').pipe(delay(1500));

forkJoin([user$, posts$]).subscribe(([user, posts]) => {
  console.log(user, posts);
});

// Salida:
// Usuario A Lista de publicaciones
```

- Espera hasta que todos los Observables hayan completado (`complete`).
- Solo el **último valor emitido** de cada Observable se compila y emite.

[🌐 Documentación Oficial RxJS - `forkJoin`](https://rxjs.dev/api/index/function/forkJoin)


## Patrones de uso típicos

- **Ejecutar múltiples solicitudes API en paralelo y resumir todos los resultados**
- **Obtener múltiples conjuntos de datos necesarios para la carga inicial de una vez**
- **Obtener todos los datos relacionados de una vez y dibujar el renderizado de pantalla de una vez**


## Ejemplos de código práctico (con UI)

Simula múltiples solicitudes API y las muestra juntas cuando todos los resultados están disponibles.

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Ejemplo práctico de forkJoin:</h3>';
document.body.appendChild(output);

// Flujos de datos ficticios
const user$ = of({ id: 1, name: 'Juan García' }).pipe(delay(2000));
const posts$ = of([{ id: 1, title: 'Publicación 1' }, { id: 2, title: 'Publicación 2' }]).pipe(delay(1500));
const weather$ = of({ temp: 22, condition: 'Soleado' }).pipe(delay(1000));

// Mensaje de carga
const loading = document.createElement('div');
loading.textContent = 'Cargando datos...';
loading.style.color = 'blue';
output.appendChild(loading);

// Emitir todo de una vez después de que todas las solicitudes completen
forkJoin({
  user: user$,
  posts: posts$,
  weather: weather$
}).subscribe(result => {
  output.removeChild(loading);

  const pre = document.createElement('pre');
  pre.textContent = JSON.stringify(result, null, 2);
  pre.style.background = '#f5f5f5';
  pre.style.padding = '10px';
  pre.style.borderRadius = '5px';
  output.appendChild(pre);

  const summary = document.createElement('div');
  summary.textContent = `Usuario: ${result.user.name}, Clima: ${result.weather.condition}, Publicaciones: ${result.posts.length}`;
  output.appendChild(summary);
});
```

- Primero se muestra la carga,
- Cuando todos los datos están disponibles, los resultados se dibujan juntos.
