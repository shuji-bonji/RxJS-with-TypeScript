---
description: "La Función de Creación defer retrasa la ejecución de una función factory de Observable hasta el momento de la suscripción. Es útil cuando quieres evaluar diferentes valores o procesamientos por cada suscripción, o cuando quieres obtener la hora actual, valores aleatorios o el estado más reciente. Explicamos las diferencias con iif() y la implementación segura de tipos en TypeScript."
---

# defer - Creación de Observable con evaluación diferida

El operador `defer` ejecuta una función factory de Observable **al momento de la suscripción** y devuelve el Observable resultante. Esto permite retrasar la creación del Observable hasta que realmente se suscriba.

## Sintaxis básica y comportamiento

```ts
import { defer, of } from 'rxjs';

const random$ = defer(() => of(Math.random()));

random$.subscribe(console.log);
random$.subscribe(console.log);

// Salida:
// 0.8727962287400634
// 0.8499299688934545
```

En este ejemplo, como `Math.random()` se evalúa por cada suscripción, se emite un valor diferente cada vez.

[🌐 Documentación Oficial RxJS - defer](https://rxjs.dev/api/index/function/defer)

## Ejemplos de uso típicos

Es útil cuando quieres realizar procesamientos que **cambian el resultado según el momento de ejecución**, como APIs, recursos externos, hora actual o números aleatorios.

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchUser(userId: number) {
  return defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );
}

fetchUser(1).subscribe(console.log);

// Salida:
// {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
```

## Ejemplo de código práctico (con UI)

`defer` es particularmente útil para procesamientos con efectos secundarios o procesamientos que generan resultados diferentes cada vez.

En el siguiente código, puedes experimentar el significado de "generar un Observable diferente cada vez que se suscribe" usando `defer`.
Es especialmente conveniente en **casos donde quieres realizar el procesamiento de obtención cada vez** en lugar de usar caché.

### ✅ 1. Generar un número aleatorio cada vez
```ts
import { defer, of } from 'rxjs';

// Observable que genera un número aleatorio
const randomNumber$ = defer(() => {
  const random = Math.floor(Math.random() * 100);
  return of(random);
});

// Crear elementos UI
const randomContainer = document.createElement('div');
randomContainer.innerHTML = '<h3>Generación de valor aleatorio con defer:</h3>';
document.body.appendChild(randomContainer);

// Botón de generación
const generateButton = document.createElement('button');
generateButton.textContent = 'Generar valor aleatorio';
randomContainer.appendChild(generateButton);

// Área de visualización de historial
const randomHistory = document.createElement('div');
randomHistory.style.marginTop = '10px';
randomHistory.style.padding = '10px';
randomHistory.style.border = '1px solid #ddd';
randomHistory.style.maxHeight = '200px';
randomHistory.style.overflowY = 'auto';
randomContainer.appendChild(randomHistory);

// Evento del botón
generateButton.addEventListener('click', () => {
  randomNumber$.subscribe(value => {
    const entry = document.createElement('div');
    entry.textContent = `Valor generado: ${value}`;
    entry.style.padding = '5px';
    entry.style.margin = '2px 0';
    entry.style.backgroundColor = '#f5f5f5';
    entry.style.borderRadius = '3px';
    randomHistory.insertBefore(entry, randomHistory.firstChild);
  });
});

// Texto explicativo
const randomExplanation = document.createElement('p');
randomExplanation.textContent = 'Cada vez que hagas clic en el botón "Generar valor aleatorio", se generará un nuevo valor aleatorio. Cuando usas of normal, el valor se genera solo una vez al principio, pero usando defer puedes generar un nuevo valor cada vez.';
randomContainer.appendChild(randomExplanation);
```

### ✅ 2. Ejecución de solicitud API cada vez

`defer` genera un nuevo Observable cada vez que se suscribe, por lo que **es particularmente efectivo en escenas donde quieres ejecutar diferentes solicitudes API según la entrada del usuario**.
Por ejemplo, se usa en los siguientes escenarios:

- ✅ Obtener con diferentes URLs según queries o parámetros dinámicos
- ✅ Cuando quieres **obtener los datos más recientes cada vez** sin usar caché
- ✅ Cuando quieres evaluar el procesamiento de forma diferida al ocurrir un evento

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const container = document.createElement('div');
container.innerHTML = '<h3>Solicitud API con defer:</h3>';
document.body.appendChild(container);

// Campo de entrada
const input = document.createElement('input');
input.placeholder = 'Ingrese ID de usuario';
container.appendChild(input);

// Botón de ejecución
const button = document.createElement('button');
button.textContent = 'Obtener información de usuario';
container.appendChild(button);

// Visualización de resultado
const resultBox = document.createElement('pre');
resultBox.style.border = '1px solid #ccc';
resultBox.style.padding = '10px';
resultBox.style.marginTop = '10px';
container.appendChild(resultBox);

// Evento del botón
button.addEventListener('click', () => {
  const userId = input.value.trim();
  if (!userId) {
    resultBox.textContent = 'Por favor ingrese un ID de usuario';
    return;
  }

  const user$ = defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );

  resultBox.textContent = 'Cargando...';
  user$.subscribe({
    next: (data) => (resultBox.textContent = JSON.stringify(data, null, 2)),
    error: (err) => (resultBox.textContent = `Error: ${err.message}`),
  });
});
```

En este ejemplo, usando `defer` hacemos que `ajax.getJSON()` se llame en el momento en que el usuario presiona el botón,
**a diferencia de evaluar desde el principio como `of(ajax.getJSON(...))`, tenemos control completo sobre el momento de ejecución**.
