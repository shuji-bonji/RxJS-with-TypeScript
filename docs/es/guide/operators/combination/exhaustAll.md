---
description: "El operador exhaustAll ignora nuevos Observables internos mientras uno está en ejecución: Esencial para prevenir doble clic, envíos duplicados y button mashing"
titleTemplate: ':title | RxJS'
---

# exhaustAll - Ignorar Durante Activo

El operador `exhaustAll` toma un **Higher-order Observable** (Observable de Observables),
**ignora nuevos Observables internos** si un Observable interno está en ejecución.

## 🔰 Sintaxis Básica y Uso

```ts
import { fromEvent, interval } from 'rxjs';
import { map, exhaustAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Comenzar un nuevo contador para cada clic (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Ignorar nuevos clics si el contador está en ejecución
higherOrder$
  .pipe(exhaustAll())
  .subscribe(x => console.log(x));

// Salida (con 3 clics consecutivos):
// 0 (1er contador)
// 1 (1er contador)
// ← Clic aquí (ignorado: el 1ro está en ejecución)
// 2 (1er contador) ← Completo
// ← Clic aquí (aceptado: sin contador en ejecución)
// 0 (2do contador)
// 1 (2do contador)
// 2 (2do contador)
```

- Si el Observable interno está en ejecución, **nuevos Observables internos se ignoran**
- **Acepta el siguiente después de que** el Observable en ejecución completa
- Ideal para prevenir ejecución doble

[🌐 Documentación Oficial de RxJS - `exhaustAll`](https://rxjs.dev/api/index/function/exhaustAll)

## 💡 Patrones de Uso Típicos

- **Prevención de doble clic (prevenir button mashing)**
- **Prevenir solicitudes de inicio de sesión duplicadas**
- **Prevenir operaciones de guardado duplicadas**

## 🧠 Ejemplo de Código Práctico

Ejemplo de prevención de doble clic en botón de guardado

```ts
import { fromEvent, of } from 'rxjs';
import { map, exhaustAll, delay } from 'rxjs';

const saveButton = document.createElement('button');
saveButton.textContent = 'Guardar';
document.body.appendChild(saveButton);

const output = document.createElement('div');
document.body.appendChild(output);

let saveCount = 0;

// Evento de clic de botón
const clicks$ = fromEvent(saveButton, 'click');

// Higher-order Observable: Operación de guardado simulada para cada clic
const saves$ = clicks$.pipe(
  map(() => {
    const id = ++saveCount;
    const start = Date.now();

    // Deshabilitar botón temporalmente (retroalimentación visual)
    saveButton.disabled = true;

    // Operación de guardado simulada (retraso de 2 segundos)
    return of(`Guardado completado #${id}`).pipe(
      delay(2000),
      map(msg => {
        saveButton.disabled = false;
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed} segundos)`;
      })
    );
  }),
  exhaustAll() // Ignorar nuevos clics mientras se guarda
);

saves$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});

// Registrar clics ignorados
clicks$.subscribe(() => {
  if (saveButton.disabled) {
    console.log('Clic ignorado durante operación de guardado');
  }
});
```

- **Nuevos clics se ignoran** durante la operación de guardado
- El siguiente clic se acepta después de que el guardado completa

## 🔄 Operadores Relacionados

| Operador | Descripción |
|---|---|
| `exhaustMap` | Atajo para `map` + `exhaustAll` (comúnmente usado) |
| [mergeAll](/es/guide/operators/combination/mergeAll) | Suscribirse a todos los Observables internos en paralelo |
| [concatAll](/es/guide/operators/combination/concatAll) | Suscribirse a Observables internos en orden (ponerlos en cola) |
| [switchAll](/es/guide/operators/combination/switchAll) | Cambiar a nuevo Observable interno (cancelar antiguo) |

## 🔄 Comparación con Otros Operadores

| Operador | Cuando Se Emite Nuevo Observable Interno |
|---|---|
| `mergeAll` | Ejecutar concurrentemente |
| `concatAll` | Agregar a cola (esperar completación anterior) |
| `switchAll` | Cancelar antiguo y cambiar |
| `exhaustAll` | **Ignorar (esperar completación en ejecución)** |

## ⚠️ Notas Importantes

### Pérdida de Eventos

`exhaustAll` **ignora completamente** eventos en ejecución, por lo que es inapropiado si desea procesar todos los eventos.

```ts
// ❌ exhaustAll es inapropiado si desea registrar todos los clics
// ✅ Use mergeAll o concatAll
```

### Retroalimentación de UI

Es importante decirle visualmente a los usuarios que los eventos están siendo "ignorados".

```ts
// Deshabilitar botón
saveButton.disabled = true;

// Mostrar mensaje toast
showToast('Procesando. Por favor espere un momento.');
```

### Casos de Uso Apropiados

#### `exhaustAll` es Óptimo para:
- Operaciones de inicio de sesión (prevenir envíos duplicados)
- Operaciones de guardado (prevenir ejecución duplicada)
- Animaciones (no iniciar nueva animación mientras se ejecuta)

#### `exhaustAll` No es Apropiado para:
- Operaciones de búsqueda (desea ejecutar última búsqueda → `switchAll`)
- Todos los eventos deben procesarse (→ `mergeAll` o `concatAll`)
