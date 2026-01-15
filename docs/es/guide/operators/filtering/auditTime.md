---
description: auditTime es un operador de filtrado de RxJS que espera un tiempo especificado después de que se emita un valor y emite el último valor dentro de ese período. Es ideal cuando deseas muestrear periódicamente el último estado de eventos de alta frecuencia como seguimiento de posición de desplazamiento, redimensionamiento de ventana y movimiento del mouse. Es importante entender la diferencia con throttleTime y debounceTime y usarlos apropiadamente.
titleTemplate: ':title | RxJS'
---

# auditTime - Último Tras Periodo

El operador `auditTime` espera un **tiempo especificado** después de que se emita un valor y emite el **último valor** dentro de ese período de tiempo. Luego espera el siguiente valor.


## 🔰 Sintaxis Básica y Uso

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('¡Clic!'));
```

**Flujo de operación**:
1. Ocurre el primer clic
2. Esperar 1 segundo (los clics durante este tiempo se registran pero no se emiten)
3. Emitir el último clic después de 1 segundo
4. Esperar el siguiente clic

[🌐 Documentación Oficial de RxJS - `auditTime`](https://rxjs.dev/api/operators/auditTime)


## 🚀 Próximos Pasos

- **[throttleTime](/es/guide/operators/filtering/throttleTime)** - Aprender cómo pasar el primer valor
- **[debounceTime](/es/guide/operators/filtering/debounceTime)** - Aprender cómo emitir valores después de que se detenga la entrada
- **[filter](/es/guide/operators/filtering/filter)** - Aprender cómo filtrar basándose en condiciones
- **[Ejemplos Prácticos de Operadores de Filtrado](/es/guide/operators/filtering/practical-use-cases)** - Aprender casos de uso reales
