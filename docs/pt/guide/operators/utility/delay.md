---
description: O operador delay atrasa o timing de emissão de cada valor no Observable por uma quantidade especificada de tempo, sendo eficaz para direção de UI e controle assíncrono.
---

# delay - Atraso de Valor

O operador `delay` é usado para atrasar a emissão de cada valor em um stream por uma quantidade especificada de tempo.
Isso é útil para encenar animações e ajustar o timing de exibição de feedback ao usuário.


## 🔰 Sintaxe Básica e Operação

Esta é a configuração mínima para emitir um valor após um determinado tempo.

```ts
import { of } from 'rxjs';
import { delay } from 'rxjs';

of('Olá')
  .pipe(
    delay(1000) // Emite valor após 1 segundo
  )
  .subscribe(console.log);
// Saída:
// Olá
```

Neste exemplo, o valor criado por `of('Olá')` é recebido por `subscribe()` com um atraso de 1 segundo.

[🌐 Documentação Oficial do RxJS - delay](https://rxjs.dev/api/index/function/delay)

## 💡 Exemplo de Uso Típico

Este é um exemplo de uso de delay para ajustar o timing de emissão em uma situação onde vários valores são emitidos.

```ts
import { of } from 'rxjs';
import { delay, concatMap } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    concatMap(
      (val, index) => of(val).pipe(delay(1000 * index)) // A imediatamente, B após 1 segundo, C após 2 segundos
    )
  )
  .subscribe(console.log);
// Saída:
// A
// B
```

Desta forma, também é possível definir um atraso separado para cada valor combinando-o com `concatMap`.


## 🧪 Exemplo de Código Prático (com UI)

```ts
import { of } from 'rxjs';
import { delay, tap } from 'rxjs';

// Área de exibição de saída
const delayOutput = document.createElement('div');
delayOutput.innerHTML = '<h3>Exemplo de delay:</h3>';
document.body.appendChild(delayOutput);

// Função para exibir hora atual
function addTimeLog(message: string) {
  const now = new Date();
  const time =
    now.toLocaleTimeString('pt-BR', { hour12: false }) +
    '.' +
    now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `${time}: ${message}`;
  delayOutput.appendChild(logItem);
}

// Registrar hora de início
addTimeLog('Início');

// Sequência de valores
of('A', 'B', 'C')
  .pipe(
    tap((val) => addTimeLog(`Antes do valor ${val} ser emitido`)),
    delay(1000), // Atraso de 1 segundo
    tap((val) => addTimeLog(`Valor ${val} emitido após 1 segundo`))
  )
  .subscribe();
```


## ✅ Resumo

- `delay` é um operador para **controlar o timing de saída do Observable**
- Pode ser combinado com `concatMap` para **controlar atraso por valor**
- Útil para **ajustes assíncronos** para melhorar UX, como saída para UI e direção de timer
