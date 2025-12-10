---
description: switchMap es un operador de transformación que cancela el Observable anterior y cambia al más reciente. Es óptimo para casos de uso como búsqueda en vivo, cambio de navegación y guardado automático, logrando procesamiento asíncrono seguro junto con la inferencia de tipos de TypeScript. También explicamos en detalle las diferencias con mergeMap y concatMap.
---

# switchMap - Cancelar el Observable anterior y cambiar al más reciente

El operador `switchMap` genera un nuevo Observable para cada valor del stream de entrada, **cancela el Observable anterior y cambia solo al Observable más reciente**.
Es óptimo para casos como formularios de búsqueda donde solo quieres que la entrada más reciente sea válida.

## 🔰 Sintaxis básica y uso

```ts
import { of } from 'rxjs';
import { delay, switchMap } from 'rxjs';

of('A', 'B', 'C').pipe(
  switchMap(value =>
    of(`${value} completado`).pipe(delay(1000))
  )
).subscribe(console.log);

// Ejemplo de salida:
// C completado
```

- Crea un nuevo Observable para cada valor.
- Sin embargo, **cuando llega un nuevo valor, el Observable anterior se cancela**.
- Finalmente, solo se emite `C`.

[🌐 Documentación Oficial RxJS - `switchMap`](https://rxjs.dev/api/operators/switchMap)

## 💡 Patrones de uso típicos

- Autocompletado de formularios de entrada
- Funcionalidad de búsqueda en vivo (solo la entrada más reciente es válida)
- Carga de recursos al cambiar navegación o enrutamiento
- Cuando quieres cambiar las acciones del usuario a la más reciente

## 🧠 Ejemplo de código práctico (con UI)

Cuando escribes caracteres en el cuadro de búsqueda, se envía inmediatamente una solicitud API y **solo se muestra el resultado de lo último ingresado**.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

// Crear campo de entrada
const searchInput = document.createElement('input');
searchInput.placeholder = 'Buscar por nombre de usuario';
document.body.appendChild(searchInput);

// Área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Procesamiento de eventos de entrada
fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  map(event => (event.target as HTMLInputElement).value.trim()),
  switchMap(term => {
    if (term === '') {
      return of([]);
    }
    return ajax.getJSON(`https://jsonplaceholder.typicode.com/users?username_like=${term}`);
  })
).subscribe(users => {
  output.innerHTML = '';

  (users as any[]).forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.username;
    output.appendChild(div);
  });
});
```

- Cada vez que cambia la entrada, la solicitud anterior se cancela.
- Solo se muestran los usuarios que coinciden con la palabra de búsqueda más reciente.
