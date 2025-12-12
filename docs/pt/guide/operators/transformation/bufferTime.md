---
description: "operador bufferTime coleta valores em intervalos de tempo regulares para processamento em lote: Perfeito para agregar eventos, logs e fluxos de dados em tempo real"
titleTemplate: ':title | RxJS'
---

# bufferTime - Gera Valores Coletados em Intervalos Regulares

O operador `bufferTime` gera **um array de valores** em intervalos de tempo especificados.
Isso é útil quando você deseja separar o stream por um certo período de tempo e tratá-lo como um processo em lote.

## 🔰 Sintaxe Básica e Uso

```ts
import { interval } from 'rxjs';
import { bufferTime } from 'rxjs';

// Emite valores a cada 100ms
const source$ = interval(100);

source$.pipe(
  bufferTime(1000)
).subscribe(buffer => {
  console.log('Valores coletados em 1 segundo:', buffer);
});

// Exemplo de saída:
// Valores coletados em 1 segundo: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Valores coletados em 1 segundo: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
// ...
```

- Valores emitidos em um segundo são agrupados em um array e gerados em sequência.

[🌐 Documentação Oficial RxJS - `bufferTime`](https://rxjs.dev/api/operators/bufferTime)

## 💡 Padrões Típicos de Uso

- Enviar lotes em intervalos regulares
- Processar operações do usuário em lotes (ex: operações de arrastar)
- Coletar dados de sensores e dispositivos IoT
- Reduzir e comprimir informações de log e rastreamento

## 🧠 Exemplo de Código Prático (com UI)

Fazer buffer de eventos de clique por 1 segundo e gerá-los juntos a cada segundo.

```ts
import { fromEvent } from 'rxjs';
import { bufferTime } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Stream de evento de clique
const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  bufferTime(1000)
).subscribe(clickArray => {
  const message = `Cliques em 1 segundo: ${clickArray.length}`;
  console.log(message);
  output.textContent = message;
});
```

- O número de cliques por segundo é exibido como um resumo.
- O processo de buffering permite gerenciar ocorrências sucessivas de eventos juntas.
