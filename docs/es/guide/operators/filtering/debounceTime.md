---
description: El operador debounceTime emite el último valor cuando no se ha recibido ningún valor nuevo durante un tiempo especificado después de emitir eventos consecutivos. Esto es ideal para optimizar entradas frecuentes como escribir en un cuadro de búsqueda o eventos de cambio de tamaño de ventana.
titleTemplate: ':title'
---

# debounceTime - Ultimo valor tras silencio

El operador `debounceTime` emite el último valor después de que un valor ha sido emitido en el flujo si no se ha emitido ningún valor nuevo durante el tiempo especificado.
Se utiliza muy comúnmente en situaciones donde es necesario suprimir eventos frecuentes, como cuadros de búsqueda de entrada de usuario.

## 🔰 Sintaxis Básica y Uso

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const searchBox = document.createElement('input');
document.body.appendChild(searchBox);

fromEvent(searchBox, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value),
    debounceTime(300)
  )
  .subscribe(console.log);
```

- Si no se recibe más entrada dentro de 300ms después de que ocurre un evento de entrada, se emite el valor.
- Esto tiene el efecto de consolidar eventos que ocurren consecutivamente en un corto período de tiempo.

[🌐 Documentación Oficial de RxJS - `debounceTime`](https://rxjs.dev/api/operators/debounceTime)

## 💡 Patrones de Uso Típicos

- Enviar solicitud después de que el usuario termine de escribir en el cuadro de búsqueda
- Obtener tamaño final para evento de cambio de tamaño de ventana
- Obtener posición final para evento de desplazamiento

## 🧠 Ejemplo de Código Práctico (con UI)

Cuando se ingresa un carácter en el cuadro de búsqueda, se muestra un mensaje de inicio de búsqueda cuando la entrada se detiene durante 300 ms.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

// Crear área de salida
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Ingrese palabra de búsqueda';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Flujo de entrada
fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300)
).subscribe(value => {
  resultArea.textContent = `Iniciada búsqueda de "${value}"`;
});
```

- No hay respuesta inmediata mientras se ingresa
- Dejará de ingresar e iniciará la búsqueda con el último valor de entrada 300ms después
