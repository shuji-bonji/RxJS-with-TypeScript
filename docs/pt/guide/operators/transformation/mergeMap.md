---
description: O operador mergeMap converte cada valor em um novo Observable, executa-os simultaneamente e combina-os de forma plana. Isso é útil quando várias requisições de API precisam ser executadas em paralelo sem esperar em sequência, ou para gerenciar processamento assíncrono aninhado.
titleTemplate: ':title'
---

# mergeMap - Fusao paralela

O operador `mergeMap` (também conhecido como `flatMap`) converte cada valor em um novo Observable e **os mescla de forma plana simultaneamente**.
É muito útil quando você deseja executar requisições imediatamente sem esperar em sequência, ou para processamento assíncrono aninhado.

## 🔰 Sintaxe Básica e Uso

```ts
import { of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  mergeMap(value =>
    of(`${value} concluído`).pipe(delay(1000))
  )
).subscribe(console.log);

// Exemplo de saída (sem ordem específica):
// A concluído
// B concluído
// C concluído
```

- Gera um novo Observable para cada valor.
- Esses Observables são **executados em paralelo** e os resultados são gerados sem ordem específica.

[🌐 Documentação Oficial RxJS - `mergeMap`](https://rxjs.dev/api/operators/mergeMap)

## 💡 Padrões Típicos de Uso

- Lançar requisição de API para cada clique de botão
- Iniciar upload de arquivo para cada evento de soltar arquivo
- Disparar tarefas assíncronas simultaneamente acionadas por operações do usuário

## 🧠 Exemplo de Código Prático (com UI)

Este é um exemplo de disparo de uma requisição assíncrona (resposta após 2 segundos) cada vez que um botão é clicado.

```ts
import { fromEvent, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

// Criar botão
const button = document.createElement('button');
button.textContent = 'Enviar Requisição';
document.body.appendChild(button);

// Área de saída
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Evento de clique
fromEvent(button, 'click').pipe(
  mergeMap((_, index) => {
    const requestId = index + 1;
    console.log(`Requisição ${requestId} iniciada`);
    return of(`Resposta ${requestId}`).pipe(delay(2000));
  })
).subscribe((response) => {
  const div = document.createElement('div');
  div.textContent = `✅ ${response}`;
  output.appendChild(div);
});
```

- Com cada clique, uma requisição assíncrona é emitida imediatamente.
- **Espera 2 segundos para cada requisição individualmente**, então os resultados não são ordenados por ordem de chegada.
- Este é um ótimo exemplo para entender processamento paralelo.
