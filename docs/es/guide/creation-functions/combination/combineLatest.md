---
description: "combineLatest combina los valores más recientes de múltiples Observables: Esencial para validación de formularios en tiempo real, sincronización de estado y datos dependientes"
---

# combineLatest - combinar los valores más recientes

`combineLatest` es una Función de Creación que **combina todos los valores más recientes de múltiples Observables**.
Cada vez que se emite un nuevo valor desde cualquiera de los Observables fuente, se combinan todos los valores más recientes.

## Sintaxis básica y uso

```ts
import { combineLatest, of } from 'rxjs';

const obs1$ = of('A', 'B', 'C');
const obs2$ = of(1, 2, 3);

combineLatest([obs1$, obs2$]).subscribe(([val1, val2]) => {
  console.log(val1, val2);
});

// Salida:
// C 1
// C 2
// C 3
```

- Después de que cada Observable ha emitido **al menos un valor**, se emite el valor combinado.
- Cada vez que llega un nuevo valor de cualquiera, se re-emite el par más reciente.

[🌐 Documentación Oficial RxJS - `combineLatest`](https://rxjs.dev/api/index/function/combineLatest)


## Patrones de uso típicos

- **Validación en tiempo real de entradas de formulario** (ej., monitoreo simultáneo de nombre y correo electrónico)
- **Sincronización de estado de múltiples flujos** (ej., integración de valores de sensores y estado del dispositivo)
- **Obtención de datos con dependencias** (ej., combinación de ID de usuario e ID de configuración)

## Ejemplos de código práctico (con UI)

Siempre combina y muestra el estado más reciente de los dos campos de entrada de un formulario.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Ejemplo práctico de combineLatest:</h3>';
document.body.appendChild(output);

// Crear campos de formulario
const nameInput = document.createElement('input');
nameInput.placeholder = 'Ingresa nombre';
document.body.appendChild(nameInput);

const emailInput = document.createElement('input');
emailInput.placeholder = 'Ingresa correo';
document.body.appendChild(emailInput);

// Observable de cada entrada
const name$ = fromEvent(nameInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

const email$ = fromEvent(emailInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

// Combinar los valores de entrada más recientes
combineLatest([name$, email$]).subscribe(([name, email]) => {
  output.innerHTML = `
    <div><strong>Nombre:</strong> ${name}</div>
    <div><strong>Correo:</strong> ${email}</div>
  `;
});
```

- Cuando escribes en cualquier campo, los **dos estados de entrada más recientes** se muestran inmediatamente.
- Se usa `startWith('')` para obtener el resultado combinado desde el principio.


## Operadores Relacionados

- **[combineLatestWith](/es/guide/operators/combination/combineLatestWith)** - Versión Pipeable Operator (usado en pipeline)
- **[withLatestFrom](/es/guide/operators/combination/withLatestFrom)** - solo el flujo principal dispara
- **[zip](/es/guide/creation-functions/combination/zip)** - Función de Creación que empareja valores correspondientes
