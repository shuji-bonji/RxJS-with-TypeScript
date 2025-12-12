---
description: ConcatMap é um operador de conversão que processa cada Observable por vez e espera o próximo até que o anterior seja concluído, ideal para cenários onde a ordem de execução é importante, como execução serial de chamadas de API ou garantias de ordem de upload de arquivos. A inferência de tipo TypeScript permite encadeamento assíncrono type-safe, e as diferenças de mergeMap e switchMap também são explicadas.
---

# concatMap - Executa Cada Observable em Ordem

O operador `concatMap` converte cada valor no stream de entrada em um Observable e **executa e concatena-os por vez**.
Ele não inicia o próximo Observable **até que o Observable anterior seja concluído**.

## 🔰 Sintaxe Básica e Uso

```ts
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  concatMap(value =>
    of(`${value} concluído`).pipe(delay(1000))
  )
).subscribe(console.log);

// Saída (em ordem):
// A concluído
// B concluído
// C concluído
```
- Converte cada valor em um Observable.
- O próximo Observable é executado após a conclusão do Observable anterior.

[🌐 Documentação Oficial RxJS - concatMap](https://rxjs.dev/api/index/function/concatMap)

## 💡 Padrões Típicos de Uso
- Execução de requisições de API críticas para ordem
- Processamento de tarefas baseado em fila
- Controle de animações e UI passo a passo
- Processo de envio de mensagens onde a ordem de envio é importante


## 🧠 Exemplo de Código Prático (com UI)

Este é um exemplo onde uma requisição é gerada cada vez que um botão é clicado e as requisições são sempre processadas em ordem.

```ts
import { fromEvent, of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

// Criar botão
const button = document.createElement('button');
button.textContent = 'Enviar Requisição';
document.body.appendChild(button);

// Área de saída
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Evento de clique
fromEvent(button, 'click')
  .pipe(
    concatMap((_, index) => {
      const requestId = index + 1;
      console.log(`Requisição ${requestId} iniciada`);
      return of(`Resposta ${requestId}`).pipe(delay(2000));
    })
  )
  .subscribe((response) => {
    const div = document.createElement('div');
    div.textContent = `✅ ${response}`;
    output.appendChild(div);
  });

```

- Cada requisição é sempre enviada e concluída em ordem.
- A próxima requisição é emitida após a conclusão da requisição anterior.
